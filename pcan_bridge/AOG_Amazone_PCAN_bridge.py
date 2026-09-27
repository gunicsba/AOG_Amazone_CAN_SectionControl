"""
AOG-Amazone-PCAN Bridge: AgOpenGPS <-> Amazone AMATRON section control
through a PEAK PCAN-USB adapter, no Arduino needed.

Towards AgOpenGPS it behaves like the AOG-TaskController (ISOBUS section
control PGNs), so AOG shows its ISOBUS section control button:

  button ON  -> autosection: AOG's section states are sent to the AMATRON
                as AmaClick commands (CAN ID 0x18E6FFCE)
  button OFF -> marking: the operator switches sections on the AMATRON,
                the bridge only reports them back so AOG paints coverage

In both modes AOG paints coverage from the section states the AMATRON
reports (show/hide messages on CAN ID 0x1CE72687), so the map shows what
the sprayer really does.

AOG -> bridge (UDP 8888):  0xF1 section control on/off request,
                           0xE5 64 section states (0xFE fallback)
bridge -> AOG (UDP 9999):  0xF0 heartbeat every 100 ms
                           [enabled|clients<<1, section count, state bits...]
"""
import can
import socket
import threading
import time
import msvcrt
import logging
import os
import sys
from configparser import ConfigParser
from typing import Optional

# ---------------------------------------------------------------------------
#  Timing / protocol constants
# ---------------------------------------------------------------------------
DEFAULT_SECTION_COUNT = 7
MAX_SECTIONS = 13               # AMATRON / AmaClick supports 13 sections
CAN_BITRATE = 250000            # ISOBUS

UDP_PORT = 8888
AOG_PORT = 9999
AOG_SRC = 0x7F                  # AgOpenGPS / AgIO source address
TC_SRC = 0x80                   # same source address the AOG-TaskController uses
PGN_SC_HEARTBEAT = 0xF0
PGN_SC_REQUEST = 0xF1
PGN_SECTIONS_64 = 0xE5
PGN_STEER_DATA = 0xFE

HEARTBEAT_S = 0.1               # AOG drops ISOBUS mode after 1 s without heartbeat
AMACLICK_S = 0.2                # 5 Hz like the Arduino sketches
CAN_TIMEOUT_S = 6.0             # no AMATRON status -> AMATRON is off
AOG_TIMEOUT_S = 2.0             # no section data from AOG -> leave autosection

# Our identity on the bus: the AmaClick joystick (source address 0xCE)
ID_ADDRESS_CLAIM = 0x18EEFFCE
ADDRESS_CLAIM_DATA = bytes([0x28, 0xEC, 0x44, 0x0C, 0x00, 0x80, 0x1A, 0x20])
ID_AMACLICK = 0x18E6FFCE
OUR_SA = 0xCE
PGN_REQUEST = 0xEA00
PGN_ADDRESS_CLAIM = 0xEE00

# Section status: ECU (0x87) -> VT (0x26) "hide/show object" (0xA0).
# Bytes 1-2 = object ID, byte 3 = 1 shown (section on) / 0 hidden (off).
# Sections 1-7 are known: 0x03F7, 0x03F9 ... 0x0403. Sections 8-13 are
# extrapolated from the step of 2 and not verified on a machine.
ID_AMATRON_STATUS = 0x1CE72687
VT_HIDE_SHOW = 0xA0
SECTION_OBJECT_BASE = 0x03F7
SECTION_OBJECT_STEP = 2

TICK_S = 0.05

# ---------------------------------------------------------------------------
#  Logging
# ---------------------------------------------------------------------------
LOG_LEVEL = logging.INFO


def get_app_directory() -> str:
    if getattr(sys, 'frozen', False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


APP_DIR = get_app_directory()
exe_name = os.path.splitext(os.path.basename(sys.executable if getattr(sys, 'frozen', False)
                                              else __file__))[0]
LOG_PATH = os.path.join(APP_DIR, f"{exe_name}_{time.strftime('%Y%m%d_%H%M%S')}.log")

# Console stays at INFO; the file gets DEBUG (unknown status objects, AOG values)
_console = logging.StreamHandler()
_console.setLevel(LOG_LEVEL)
_console.setFormatter(logging.Formatter(
    "[%(asctime)s.%(msecs)03d] %(levelname)s %(message)s", datefmt="%H:%M:%S"))
_file = logging.FileHandler(LOG_PATH, mode='w', encoding='utf-8')
_file.setLevel(logging.DEBUG)
_file.setFormatter(logging.Formatter(
    "[%(asctime)s.%(msecs)03d] %(levelname)s [%(threadName)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"))
logging.basicConfig(level=logging.DEBUG, handlers=[_console, _file])
logger = logging.getLogger("amatron")
logger.info(f"Logging to file: {LOG_PATH}")
# python-can warns that the optional 'uptime' package is missing; frame
# timestamps are not used here
logging.getLogger("can.pcan").setLevel(logging.ERROR)

# ---------------------------------------------------------------------------
#  Config
# ---------------------------------------------------------------------------
CONFIG_PATH = os.path.join(APP_DIR, "config.ini")


def load_config() -> ConfigParser:
    config = ConfigParser()
    if not os.path.exists(CONFIG_PATH):
        config["main"] = {
            "pcan_adapter": "1",
            "sections": str(DEFAULT_SECTION_COUNT),
            "subnet": "255.255.255.255",
        }
        with open(CONFIG_PATH, "w") as f:
            config.write(f)
    else:
        config.read(CONFIG_PATH)
    return config


# ---------------------------------------------------------------------------
#  Message helpers
# ---------------------------------------------------------------------------
def aog_checksum(msg: bytes) -> int:
    """Sum bytes 2..n-1 (everything between preamble and CRC slot)."""
    return sum(msg[2:]) & 0xFF


def build_heartbeat(enabled: bool, section_count: int, states: int) -> bytes:
    """PGN 0xF0: byte0 bit0 = SC enabled, bits1-3 = client count (1),
    byte1 = number of sections, then one bit per section."""
    payload = bytearray([(1 if enabled else 0) | (1 << 1), section_count])
    for i in range((section_count + 7) // 8):
        payload.append((states >> (8 * i)) & 0xFF)
    msg = bytearray([0x80, 0x81, TC_SRC, PGN_SC_HEARTBEAT, len(payload)]) + payload
    msg.append(aog_checksum(msg))
    return bytes(msg)


def build_amaclick(states: int, section_count: int) -> bytes:
    """AmaClick section command: byte2 = sections 1-8, byte3 bits 0-4 =
    sections 9-13, byte3 bit 7 = main switch (on when any section is on)."""
    byte2 = states & 0xFF
    byte3 = (states >> 8) & 0x1F
    if states:
        byte3 |= 0x80
    return bytes([0x21, 0xFA, byte2, byte3, 0x00, 0x00, 0x01, section_count])


def section_for_object(object_id: int, section_count: int) -> Optional[int]:
    """Map a VT object ID to a 0-based section index, or None."""
    offset = object_id - SECTION_OBJECT_BASE
    if offset < 0 or offset % SECTION_OBJECT_STEP:
        return None
    index = offset // SECTION_OBJECT_STEP
    return index if index < section_count else None


def bits(states: int, section_count: int) -> str:
    """Section 1 first, e.g. '1101000'."""
    return "".join("1" if states >> i & 1 else "0" for i in range(section_count))


# ---------------------------------------------------------------------------
#  Bridge
# ---------------------------------------------------------------------------
class Bridge:
    def __init__(self, bus: can.BusABC, section_count: int, subnet: str):
        self.bus = bus
        self.section_count = section_count
        self.section_mask = (1 << section_count) - 1
        self.aog_addr = (subnet, AOG_PORT)
        self.running = True

        self.lock = threading.Lock()
        self.can_lock = threading.Lock()
        self.sc_enabled = False         # AOG ISOBUS button: autosection
        self.desired = 0                # sections AOG wants on
        self.actual = 0                 # sections the AMATRON reports on
        self.last_aog_rx = 0.0
        self.last_can_rx = 0.0
        self.can_alive = False
        self.last_foreign_amaclick_warn = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", UDP_PORT))
        self.sock.settimeout(0.5)
        logger.info(f"UDP listening on port {UDP_PORT}, heartbeat -> {self.aog_addr}")

    # --- CAN ---------------------------------------------------------------
    def can_send(self, arbitration_id: int, data: bytes):
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        try:
            with self.can_lock:
                self.bus.send(msg, timeout=0.1)
        except can.CanError as e:
            logger.warning(f"CAN send 0x{arbitration_id:08X} failed: {e}")

    def claim_address(self):
        self.can_send(ID_ADDRESS_CLAIM, ADDRESS_CLAIM_DATA)
        logger.info("Address claim sent (0xCE, AmaClick)")

    def can_receive_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.5)
            except can.CanError as e:
                logger.warning(f"CAN receive error: {e}")
                time.sleep(0.5)
                continue
            if msg is None or not msg.is_extended_id or msg.is_error_frame:
                continue
            can_id = msg.arbitration_id
            if can_id == ID_AMATRON_STATUS:
                self.handle_status(bytes(msg.data))
            elif can_id == ID_AMACLICK:
                # PCAN does not echo our own frames: this is a real AmaClick
                # on the bus using our source address.
                now = time.monotonic()
                if now - self.last_foreign_amaclick_warn > 10:
                    self.last_foreign_amaclick_warn = now
                    logger.warning("Another AmaClick is sending on the bus -- unplug it "
                                   "or turn it off while using autosection")
            elif (can_id >> 8) & 0x3FF00 == PGN_REQUEST and can_id & 0xFF00 in (0xFF00, OUR_SA << 8):
                # Request for address claim, global or to us
                data = bytes(msg.data)
                if len(data) >= 3 and data[0] | data[1] << 8 | data[2] << 16 == PGN_ADDRESS_CLAIM:
                    self.claim_address()

    def handle_status(self, data: bytes):
        with self.lock:
            self.last_can_rx = time.monotonic()
            if not self.can_alive:
                self.can_alive = True
                logger.info("AMATRON detected on CAN")
                reclaim = True
            else:
                reclaim = False
        if reclaim:
            self.claim_address()

        if len(data) < 4 or data[0] != VT_HIDE_SHOW:
            return
        object_id = data[1] | data[2] << 8
        section = section_for_object(object_id, self.section_count)
        if section is None:
            logger.debug(f"Status: object 0x{object_id:04X} -> {data[3]} (not a known section)")
            return
        with self.lock:
            before = self.actual
            if data[3] == 1:
                self.actual |= 1 << section
            elif data[3] == 0:
                self.actual &= ~(1 << section)
            after = self.actual
        if after != before:
            logger.info(f"AMATRON sections {bits(after, self.section_count)}")

    # --- UDP from AgOpenGPS ----------------------------------------------------
    def udp_receive_loop(self):
        got_e5 = False
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                if not self.running:
                    break
                raise
            if len(data) < 6 or data[0] != 0x80 or data[1] != 0x81 or data[2] != AOG_SRC:
                continue
            pgn = data[3]
            if pgn == PGN_SC_REQUEST:
                self.set_sc_enabled(data[5] == 1)
            elif pgn == PGN_SECTIONS_64 and len(data) >= 5 + 8:
                if not got_e5:
                    logger.info("AgOpenGPS sends PGN 0xE5, using it for section states")
                got_e5 = True
                self.update_desired(int.from_bytes(data[5:13], "little"))
            elif pgn == PGN_STEER_DATA and len(data) > 12 and not got_e5:
                self.update_desired(data[11] | data[12] << 8)

    def update_desired(self, states: int):
        with self.lock:
            self.desired = states & self.section_mask
            self.last_aog_rx = time.monotonic()
        logger.debug(f"AOG sections {bits(states, self.section_count)}")

    def set_sc_enabled(self, enabled: bool):
        with self.lock:
            if self.sc_enabled == enabled:
                return
            self.sc_enabled = enabled
            if enabled:
                self.last_aog_rx = time.monotonic()
        logger.info("Mode: AUTOSECTION (AOG switches the AMATRON)" if enabled
                    else "Mode: MARKING (AMATRON switches, AOG paints)")

    # --- periodic ------------------------------------------------------------
    def periodic_loop(self):
        next_heartbeat = next_amaclick = 0.0
        while self.running:
            now = time.monotonic()
            with self.lock:
                if self.can_alive and now - self.last_can_rx > CAN_TIMEOUT_S:
                    self.can_alive = False
                    self.actual = 0
                    logger.warning("AMATRON lost (no CAN status) -- reporting all sections off")
                if self.sc_enabled and now - self.last_aog_rx > AOG_TIMEOUT_S:
                    aog_lost = True
                    self.sc_enabled = False
                else:
                    aog_lost = False
                sc_enabled, desired, actual, can_alive = (
                    self.sc_enabled, self.desired, self.actual, self.can_alive)

            if aog_lost:
                logger.warning("No section data from AgOpenGPS -- sections off, back to MARKING")
                if can_alive:
                    for _ in range(3):
                        self.can_send(ID_AMACLICK, build_amaclick(0, self.section_count))
                        time.sleep(0.05)

            if now >= next_heartbeat:
                next_heartbeat = now + HEARTBEAT_S
                try:
                    self.sock.sendto(build_heartbeat(sc_enabled, self.section_count, actual),
                                     self.aog_addr)
                except OSError as e:
                    logger.debug(f"Heartbeat send failed: {e}")

            if now >= next_amaclick:
                next_amaclick = now + AMACLICK_S
                if sc_enabled and can_alive:
                    self.can_send(ID_AMACLICK, build_amaclick(desired, self.section_count))

            time.sleep(TICK_S)

    def shutdown(self):
        """Stop the sprayer if we were switching it."""
        with self.lock:
            active = self.sc_enabled and self.can_alive
            self.sc_enabled = False
        if active:
            logger.info("Exit in autosection -- switching sections off")
            for _ in range(3):
                self.can_send(ID_AMACLICK, build_amaclick(0, self.section_count))
                time.sleep(0.05)
        self.sock.close()


def keyboard_loop(bridge: Bridge):
    """Thread: keyboard input. X = exit."""
    logger.info("Keyboard: X = exit")
    while bridge.running:
        if msvcrt.kbhit():
            if msvcrt.getch() in (b"x", b"X"):
                bridge.running = False
                logger.info("Exit requested")
                break
        time.sleep(0.05)


# ---------------------------------------------------------------------------
#  Console close (window X, logoff, shutdown)
# ---------------------------------------------------------------------------
_console_handler = None     # keep a reference, or ctypes frees the callback


def install_console_close_handler(bridge: Bridge):
    """Switch sections off when the console window is closed. Windows gives
    the process ~5 s after CTRL_CLOSE_EVENT."""
    global _console_handler
    import ctypes
    from ctypes import wintypes

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.DWORD)
    def handler(event):
        if event in (2, 5, 6):      # CLOSE, LOGOFF, SHUTDOWN
            logger.info("Console closing -- shutting down")
            bridge.running = False
            time.sleep(0.3)
            try:
                bridge.shutdown()
                bridge.bus.shutdown()
            except Exception as e:
                logger.warning(f"Shutdown on close failed: {e}")
            logging.shutdown()
            return True
        return False                # Ctrl+C: default -> KeyboardInterrupt

    _console_handler = handler
    if not ctypes.windll.kernel32.SetConsoleCtrlHandler(handler, True):
        logger.warning("Could not install console close handler")


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------
def list_pcan_channels():
    try:
        configs = can.detect_available_configs(interfaces=["pcan"])
    except Exception as e:
        logger.error(f"Could not list PCAN adapters: {e}")
        return
    if not configs:
        logger.error("No PCAN adapters found. Is the PEAK driver installed and the adapter plugged in?")
        return
    logger.info("Available PCAN adapters:")
    for c in configs:
        logger.info(f"  {c['channel']}")


def open_bus(adapter: int, interface: str) -> Optional[can.BusABC]:
    channel = f"PCAN_USBBUS{adapter}" if interface == "pcan" else str(adapter)
    try:
        bus = can.Bus(interface=interface, channel=channel, bitrate=CAN_BITRATE)
    except Exception as e:
        logger.error(f"Cannot open {channel}: {e}")
        if interface == "pcan":
            list_pcan_channels()
            logger.error("Set pcan_adapter in config.ini to the adapter number "
                         "(1 = PCAN_USBBUS1, 2 = PCAN_USBBUS2, ...)")
        return None
    logger.info(f"Opened {channel} @ {CAN_BITRATE // 1000} kbit/s")
    return bus


def main():
    print("AOG-Amazone-PCAN Bridge  (AgOpenGPS <-> AMATRON over PCAN-USB)")
    print()

    config = load_config()
    adapter = config.getint("main", "pcan_adapter", fallback=1)
    section_count = config.getint("main", "sections", fallback=DEFAULT_SECTION_COUNT)
    subnet = config.get("main", "subnet", fallback="255.255.255.255")
    interface = config.get("main", "interface", fallback="pcan")  # 'virtual' for testing

    if not 1 <= section_count <= MAX_SECTIONS:
        logger.error(f"sections = {section_count} in config.ini, the AMATRON supports 1-{MAX_SECTIONS}")
        input("Press Enter to exit")
        return
    print(f"Config: pcan_adapter={adapter}  sections={section_count}  subnet={subnet}")
    if section_count > 7:
        logger.warning("Status codes for sections 8-13 are extrapolated, not verified: "
                       "check that AOG paints those sections correctly")
    print()

    bus = open_bus(adapter, interface)
    if bus is None:
        input("Press Enter to exit")
        return

    try:
        bridge = Bridge(bus, section_count, subnet)
    except OSError as e:
        logger.error(f"Cannot open UDP port {UDP_PORT}: {e} "
                     "(is the AOG-TaskController or another bridge running?)")
        bus.shutdown()
        input("Press Enter to exit")
        return

    bridge.claim_address()
    logger.info("Mode: MARKING -- press the ISOBUS section control button in AOG for autosection")

    threads = [
        threading.Thread(target=bridge.can_receive_loop, name="can", daemon=True),
        threading.Thread(target=bridge.udp_receive_loop, name="udp", daemon=True),
        threading.Thread(target=bridge.periodic_loop, name="periodic", daemon=True),
        threading.Thread(target=keyboard_loop, args=(bridge,), name="keys", daemon=True),
    ]
    for t in threads:
        t.start()
    install_console_close_handler(bridge)

    try:
        while bridge.running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt")
    finally:
        bridge.running = False
        time.sleep(0.6)
        try:
            bridge.shutdown()
        except Exception as e:
            logger.warning(f"Shutdown failed: {e}")
        bus.shutdown()
        logger.info("CAN closed")


if __name__ == "__main__":
    main()
