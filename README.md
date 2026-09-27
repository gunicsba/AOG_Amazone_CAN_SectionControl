# AOG_Amazone_CAN_SectionControl
SectionControl based on CAN Messages for AgOpen GPS and Amazone machines

THe sketches are based on the Machine_USB_v5 Sketch from AGO... the following sketches unfortunately only support USB (sorry for that). When you need UDP -> copy code into Machine_UDP sketch
3 Sketches:
 1. This sketch supports marking of sections into AGO -> use your sprayer as usual and AGO marking will be triggered by sections of your sprayer
 2. This sketch supports AutoSection control from AGO to your sprayer -> choose Autosection Mode in AGO and the sections of Amazone will be automatically changed
 3. This sketch supports both -> you need a physical switch to change between the two modes (marking and autosection control)

See attached a components list and schematic. Also an excel file which explains the CAN Message structure.
You can use only AMATRON, or with AMACLICK and also with Joystick at the same time. Be careful when you use AMACLICK in AUtosection Mode -> turn off AMACLICK or remove it.
Attention: when using the CANBUS shield and the optional physical switch -> PIN 2,9,19,17 are not available

Be careful when using CANBUS (use the right Pins and do not current them!). Use at your own risk. I tested the code but no guarantee for none complications
At the moment November 2023 the "marking mode" (Sketch 1) only supports until 7 Sections because the CAN Code for upper sections is unknown.
Please feel free to change the code or adapt to your situation.

THANKS a lot to Valentin for support :-)

## PCAN-USB version (no Arduino)

`pcan_bridge/` has a Windows program that does the same job with a PEAK PCAN-USB adapter plugged into the AgOpenGPS computer, connected to the AMATRON CAN bus (the same Y adapter as for the CAN shield).

It talks to AgOpenGPS like the ISOBUS Task Controller does, so AgOpenGPS shows its **ISOBUS section control button**. That button picks the mode, no physical switch needed:

| ISOBUS button | Mode | What happens |
|---|---|---|
| off | Marking | You switch sections on the AMATRON / AmaClick / joystick, AgOpenGPS paints them |
| on | Autosection | AgOpenGPS switches the AMATRON sections (sections in AOG must be in Auto) |

In both modes AgOpenGPS paints coverage from the sections the AMATRON reports as on, so the map shows what really sprayed.

### Setup
1. Install the PEAK driver ([peak-system.com](https://www.peak-system.com/Drivers.523.0.html)) and plug in the PCAN-USB.
2. Download `AOG-Amazone-PCAN.exe` from the [Releases](../../releases) page. It has Python and everything else built in.
3. Run it once, it creates `config.ini` next to the exe:
   ```ini
   [main]
   pcan_adapter = 1          ; 1 = PCAN_USBBUS1, 2 = PCAN_USBBUS2, ...
   sections = 7              ; number of sections on the sprayer (max 13)
   subnet = 255.255.255.255  ; where the heartbeat to AgIO is sent
   ```
   If the adapter can't be opened, the program lists the PCAN adapters it finds.
4. Start AgIO + AgOpenGPS, then the bridge. The ISOBUS button appears in AgOpenGPS.

Don't run the AOG-TaskController (AgIO ISOBUS) at the same time, both use the same PGNs.

### Good to know
- Autosection sends AmaClick commands with source address 0xCE: turn off or unplug a real AmaClick while in autosection (the bridge warns if it sees one).
- If AgOpenGPS stops sending for 2 s in autosection, the bridge switches all sections off and goes back to marking. Closing the bridge in autosection also switches sections off.
- If the AMATRON is quiet for 6 s (turned off), all sections are reported off.
- The AMATRON only reports a section when it changes, so sections that are already on when the bridge starts show up after their next change.
- Status codes are known for sections 1-7. Sections 8-13 are guessed from the pattern and not tested yet. Unknown status objects are written to the log file, which helps finding them.

### Building yourself
`pcan_bridge/build.bat` (needs Python 3.12). A GitHub release with the exe is built automatically when a `v*` tag is pushed.
