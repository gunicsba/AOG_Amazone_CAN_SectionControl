
   /*
    * This program only turns the relays for section control
    * On and Off. Connect to the Relay Port in AgOpenGPS
    * 
    * Hydraulic Raise D4
    * Hydraulic Lower D3
    * 
    * Tram Right Side D5
    * Tram left Side D6
    * 
    * Section 0 to 5 -- D7 to D12
    * edited for Amatron CAN BUS by Alexander Beck/Valentin /Nov 2023
    */
    
  
  //loop time variables in microseconds

  #include <EEPROM.h> 
  #define EEP_Ident 0x5400
  //------------added because of CAN Amatron-------

//----Teensy 4.1 Ethernet--Start---------------------
  #include <NativeEthernet.h>
  #include <NativeEthernetUdp.h>

    struct ConfigIP {
        uint8_t ipOne = 192;
        uint8_t ipTwo = 168;
        uint8_t ipThree = 1;
    };  ConfigIP networkAddress;   //3 bytes
  
  // Module IP Address / Port
  IPAddress ip = { 0,0,0,123 };
  unsigned int localPort = 8888;

  // AOG IP Address / Port
  static uint8_t ipDestination[] = {0,0,0,255};
  unsigned int AOGPort = 9999;

  //MAC address
  byte mac[] = { 0x00,0x00,0x56,0x00,0x00,0x7E };
  
  // Buffer For Receiving UDP Data
  byte udpData[128];    // Incomming Buffer
  byte NtripData[512];   

  // An EthernetUDP instance to let us send and receive packets over UDP
  EthernetUDP Udp;

//----Teensy 4.1 Ethernet--End---------------------


    //Program counter reset
    void(* resetFunc) (void) = 0;

    //Variables for config - 0 is false  
    struct Config {
        uint8_t raiseTime = 2;
        uint8_t lowerTime = 4;
        uint8_t enableToolLift = 0;
        uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

        uint8_t user1 = 0; //user defined values set in machine tab
        uint8_t user2 = 0;
        uint8_t user3 = 0;
        uint8_t user4 = 0;

    };  Config aogConfig;   //4 bytes

//----Teensy 4.1 CANBus--Start---------------------

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> AmaBus;

  // Set Amatron Claim adress to send data
  byte AmaClick_addressClaim[8] = {0x28, 0xEC, 0x44, 0x0C, 0x00, 0x80, 0x1A, 0x20};
  //set Amatron message to change Sections
  uint8_t AmaClick_data[8] = {0x21, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x01, 0x09};
  byte AmaClick_data_byte2 = 0b00000000;
  byte AmaClick_data_byte3 = 0b00000000;  
  //----------------------

  /*
    * Functions as below assigned to pins
    0: -
    1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8, 
                Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16, 
    17,18    Hyd Up, Hyd Down, 
    19 Tramline, 
    20: Geo Stop
    21,22,23 - unused so far
    */

    //24 possible pins assigned to these functions
    uint8_t pin[] = { 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

    //read value from Machine data and set 1 or zero according to list
    uint8_t relayState[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

  //Relays
  bool isRelayActiveHigh = true;
  uint8_t relayLo = 0, relayHi = 0, uTurn = 0, hydLift = 0;
  uint8_t tram = 0;

  const uint8_t LOOP_TIME = 200; //5hz
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;
  uint32_t fifthTime = 0;
  uint16_t count = 0;

  //Comm checks
  uint8_t watchdogTimer = 0; //make sure we are talking to AOG
  uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

  bool isRaise = false;
  bool isLower = false;
  
   //Communication with AgOpenGPS
  int16_t temp, EEread = 0;

   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;
  
 // uint8_t AOG[] = {0x80,0x81, 0x7f, 0xED, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
  uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

  //settings pgn
  uint8_t PGN_237[] = { 0x80,0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0,0,0,0, 0xCC };
  uint8_t AOG[] = { 0x80,0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0,0,0,0, 0xCC };
  int8_t PGN_237_Size = sizeof(PGN_237) - 1;

  float gpsSpeed;
  
  uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;


void setup() {
  // put your setup code here, to run once:
    delay(500);                         //Small delay so serial can monitor start up
    Serial.begin(115200);
    delay (2000);
          EEPROM.get(0, EEread);              // read identifier

      if (EEread != EEP_Ident)   // check on first start and write EEPROM
      {
          EEPROM.put(0, EEP_Ident);
          EEPROM.put(6, aogConfig);
          EEPROM.put(20, pin);
          EEPROM.put(60, networkAddress);
      }
      else
      {
          EEPROM.get(6, aogConfig);
          EEPROM.get(20, pin);
          EEPROM.get(60, networkAddress);
      }

    //----Teensy 4.1 Ethernet--Start---------------------
  
      delay(500);
 
      if (Ethernet.linkStatus() == LinkOFF) 
      {
        Serial.println("\r\nEthernet cable is not connected - Who cares we will start ethernet anyway.");
      }  

      Ethernet.begin(mac,0);          // Start Ethernet with IP 0.0.0.0

      //grab the ip from EEPROM
      ip[0] = networkAddress.ipOne;
      ip[1] = networkAddress.ipTwo;
      ip[2] = networkAddress.ipThree;

      ipDestination[0] = networkAddress.ipOne;
      ipDestination[1] = networkAddress.ipTwo;
      ipDestination[2] = networkAddress.ipThree;
      
      Ethernet.setLocalIP(ip);  // Change IP address to IP set by user
      Serial.println("\r\nEthernet status OK");
      Serial.print("IP set Manually: ");
      Serial.println(Ethernet.localIP());
    
      Udp.begin(localPort);

      
    //----Teensy 4.1 Ethernet--End---------------------
   //----Teensy 4.1 CANBus--Start---------------------
    //AmaBus is CAN-3 and is the Steering BUS
    AmaBus.begin();
    AmaBus.setBaudRate(250000);
    AmaBus.enableFIFO();
    AmaBus.setFIFOFilter(REJECT_ALL);
  
    //set CAN Filter and Mask to look for sending data from AMACLICK (if AMACLICK is activ and sending we don´t want to send CAN)
    //Therfore you can chose either use manually AMACLICK or automatic Section Control
    AmaBus.setFIFOFilter(0, 0x18E6FFCE, EXT);  //Claas Curve Data & Valve State Message

    //send first Claim Adress to Amatron to send future data
    CAN_message_t msgAma;
    msgAma.id = 0x18EEFFCE;
    msgAma.flags.extended = true;
    msgAma.len = 8;
    memcpy(msgAma.buf, AmaClick_addressClaim, sizeof(AmaClick_addressClaim));//input from Valentin
    AmaBus.write(msgAma);
    delay(100);
    //Serial.println("Address Claimed");  
}

void loop() {
  // put your main code here, to run repeatedly:
        currentTime = millis();

        //--Main Timed Loop----------------------------------   
        if (currentTime - lastTime >= LOOP_TIME)
        {
          lastTime = currentTime;
          //checksum
            int16_t CK_A = 0;
            for (uint8_t i = 2; i < PGN_237_Size; i++)
            {
                CK_A = (CK_A + PGN_237[i]);
            }
            PGN_237[PGN_237_Size] = CK_A;

          Udp.beginPacket(ipDestination, AOGPort);
          Udp.write(PGN_237, sizeof(PGN_237));
          Udp.endPacket();

          SetRelays();
        } //end of main timed loop

   //Check for UDP Packet
        int packetSize = Udp.parsePacket();
        if (packetSize) {
          //Serial.println("UDP Data Avalible"); 
          udpSteerRecv(packetSize);
        }

}


void udpSteerRecv(int sizeToRead)
{
  if (sizeToRead > 128) sizeToRead = 128;
  IPAddress src_ip = Udp.remoteIP();
  Udp.read(udpData, sizeToRead);

  if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
  {
    if (udpData[3] == 0xFE)  //254
    {
      gpsSpeed = ((float)(udpData[5] | udpData[6] << 8))*0.1;


      //Serial.println(gpsSpeed); 

      //Bit 10 Tram 
      tram = udpData[10];
      
      //Bit 11
      relayLo = udpData[11];

      //Bit 12
      relayHi = udpData[12];
     
      //----------------------------------------------------------------------------
      //Serial Send to agopenGPS
      
//TODO      
      //off to AOG
      Udp.beginPacket(ipDestination, 9999);
      Udp.write(AOG, sizeof(AOG));
      Udp.endPacket();


      // Stop sending the helloAgIO message
//      helloCounter = 0;
      //--------------------------------------------------------------------------    
    }
    
    else if (udpData[3] == 200) // Hello from AgIO
    {

        if (udpData[7] == 1)
        {
          relayLo -= 255;
          relayHi -= 255;
          watchdogTimer = 0;
        }
        helloFromMachine[5] = relayLo;
        helloFromMachine[6] = relayHi;

       Udp.beginPacket(ipDestination, 9999);
       Udp.write(helloFromMachine, sizeof(helloFromMachine));
       Udp.endPacket();

    }
          
//Machine Data
    else if (udpData[3] == 0xEF)  //239 Machine Data
    {
      hydLift = udpData[7];

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }

//Machine Settings
    else if (udpData[3] == 0xEE) //238 Machine Settings 
    {         
      aogConfig.raiseTime = udpData[5];
      aogConfig.lowerTime = udpData[6];  
      //aogConfig.enableToolLift = udpData[7]; //This is wrong AgOpen is putting enable in sett,1
      
      //set1 
      uint8_t sett = udpData[8];  //setting0     
      if (bitRead(sett,0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;  
      if (bitRead(sett,1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;  

      //crc
      //udpData[13];        //crc
  
      //save in EEPROM and restart
      EEPROM.put(6, aogConfig);

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0;
    }

    else if (udpData[3] == 201)
    {
     //make really sure this is the subnet pgn
     if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
     {
      networkAddress.ipOne = udpData[7];
      networkAddress.ipTwo = udpData[8];
      networkAddress.ipThree = udpData[9];

      //save in EEPROM and restart
      EEPROM.put(60, networkAddress);
      SCB_AIRCR = 0x05FA0004; //Teensy Reset
      }
    }//end 201

    //Who Am I ?
    else if (udpData[3] == 202)
    {
     //make really sure this is the reply pgn
     if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
     {
      //hello from AgIO
      uint8_t scanReply[] = { 128, 129, 123, 203, 7,
      ip[0], ip[1], ip[2], 123, src_ip[0], src_ip[1], src_ip[2], 23 };


      //checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
       {
        CK_A = (CK_A + scanReply[i]);
       }
      scanReply[sizeof(scanReply)-1] = CK_A;

      static uint8_t ipDest[] = { 255,255,255,255 };
      uint16_t portDest = 9999; //AOG port that listens

      //off to AOG
       Udp.beginPacket(ipDest, portDest);
       Udp.write(scanReply, sizeof(scanReply));
       Udp.endPacket();
/*
       Serial.print("\r\nAdapter IP: ");
       Serial.print(src_ip[0]); Serial.print(" . ");
       Serial.print(src_ip[1]); Serial.print(" . ");
       Serial.print(src_ip[2]); Serial.print(" . ");
       Serial.print(src_ip[3]);

       Serial.print("\r\nModule  IP: ");
       Serial.print(ip[0]); Serial.print(" . ");
       Serial.print(ip[1]); Serial.print(" . ");
       Serial.print(ip[2]); Serial.print(" . ");
       Serial.print(ip[3]); Serial.println();
*/

      }
     }//end 202
    
  } //end if 80 81 7F

} //end udp callback


void SetRelays(void)
  {
      //pin, rate, duration  130 pp meter, 3.6 kmh = 1 m/sec or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
      //gpsSpeed is 10x actual speed so 3.61111
      gpsSpeed *= 3.61111;
      //tone(13, gpsSpeed);
      
      //Load the current pgn relay state - Sections
      for (uint8_t i = 0; i < 8; i++)
      {
          relayState[i] = bitRead(relayLo, i);
      }
      
      for (uint8_t i = 0; i < 8; i++)
      {
          relayState[i + 8] = bitRead(relayHi, i);
      }

      // Hydraulics
      relayState[16] = isLower;
      relayState[17] = isRaise;

      //Tram
      relayState[18] = bitRead(tram, 0); //right
      relayState[19] = bitRead(tram, 1); //left

      //GeoStop
//      relayState[20] =  (geoStop == 0) ? 0 : 1;

      //------------added because of CAN Amatron-------------------------------------
      //define data for message to send to control Amatron sections  
      //See ExcelFile or als Valentin for mor details according to Send Message
      
      //check if minimum one of the section relay is on/activ -> Main switch in Amatron on/off
      //16 relays are defined in relay[0-15]
      int isOnerelayPositiv = 0;
      for (uint8_t i = 0; i < 16; i++)
      {
          isOnerelayPositiv = isOnerelayPositiv + relayState[i];
      }
      if (isOnerelayPositiv != 0) bitWrite(AmaClick_data_byte3, 7, 1); //byte 3, bit 7 defines MainSwitch
      if (isOnerelayPositiv == 0) bitWrite(AmaClick_data_byte3, 7, 0);
      
      //write relayState into CAN message
      if (relayState[0]==HIGH) bitWrite(AmaClick_data_byte2, 0, 1);   // byte 2 and part of byte3 defines sections
      if (relayState[1]==HIGH) bitWrite(AmaClick_data_byte2, 1, 1);
      if (relayState[2]==HIGH) bitWrite(AmaClick_data_byte2, 2, 1);
      if (relayState[3]==HIGH) bitWrite(AmaClick_data_byte2, 3, 1);
      if (relayState[4]==HIGH) bitWrite(AmaClick_data_byte2, 4, 1);
      if (relayState[5]==HIGH) bitWrite(AmaClick_data_byte2, 5, 1);
      if (relayState[6]==HIGH) bitWrite(AmaClick_data_byte2, 6, 1);
      if (relayState[7]==HIGH) bitWrite(AmaClick_data_byte2, 7, 1);
      if (relayState[8]==HIGH) bitWrite(AmaClick_data_byte3, 0, 1);
      if (relayState[9]==HIGH) bitWrite(AmaClick_data_byte3, 1, 1);
      if (relayState[10]==HIGH) bitWrite(AmaClick_data_byte3, 2, 1);
      if (relayState[11]==HIGH) bitWrite(AmaClick_data_byte3, 3, 1);
      if (relayState[12]==HIGH) bitWrite(AmaClick_data_byte3, 4, 1);
      //AMATRON supports max 13 sections
      
      if (relayState[0]==LOW) bitWrite(AmaClick_data_byte2, 0, 0);
      if (relayState[1]==LOW) bitWrite(AmaClick_data_byte2, 1, 0);
      if (relayState[2]==LOW) bitWrite(AmaClick_data_byte2, 2, 0);
      if (relayState[3]==LOW) bitWrite(AmaClick_data_byte2, 3, 0);
      if (relayState[4]==LOW) bitWrite(AmaClick_data_byte2, 4, 0);
      if (relayState[5]==LOW) bitWrite(AmaClick_data_byte2, 5, 0);
      if (relayState[6]==LOW) bitWrite(AmaClick_data_byte2, 6, 0);
      if (relayState[7]==LOW) bitWrite(AmaClick_data_byte2, 7, 0);
      if (relayState[8]==LOW) bitWrite(AmaClick_data_byte3, 0, 0);
      if (relayState[9]==LOW) bitWrite(AmaClick_data_byte3, 1, 0);
      if (relayState[10]==LOW) bitWrite(AmaClick_data_byte3, 2, 0);
      if (relayState[11]==LOW) bitWrite(AmaClick_data_byte3, 3, 0);
      if (relayState[12]==LOW) bitWrite(AmaClick_data_byte3, 4, 0);

      //assign byte2 and 3 to the CAN message
      AmaClick_data[2]= AmaClick_data_byte2;
      AmaClick_data[3]= AmaClick_data_byte3;
      //check if AMACLICK is active if not then send CAN message
      if (watchdogTimer < 10){  //when connection to AGO is lost then send nothing
        CAN_message_t AmaBusReceiveData;
        AmaBus.read(AmaBusReceiveData); //read CAN with Filter and Mask for AMACLICK ID
        if (AmaBusReceiveData.buf[6]!= 1) {
         //send first Claim Adress to Amatron to send future data
          CAN_message_t msgAma;
          msgAma.id = 0x18E6FFCE;
          msgAma.flags.extended = true;
          msgAma.len = 8;
          memcpy(msgAma.buf, AmaClick_data, sizeof(AmaClick_data));
          AmaBus.write(msgAma);
        }
          Serial.print(AmaBusReceiveData.buf[6]);
          Serial.print(AmaClick_data_byte2,HEX);
          Serial.print(AmaClick_data_byte3,HEX);
          Serial.println();
      }
  }