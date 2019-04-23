/*
   LoRaLib Receive with Inerrupts Example

   This example listens for LoRa transmissions and tries to
   receive them. Once a packet is received, an interrupt is
   triggered. To successfully receive data, the following
   settings have to be the same on both transmitter 
   and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word
    - preamble length

   For more detailed information, see the LoRaLib Wiki
   https://github.com/jgromes/LoRaLib/wiki
*/

// include the library
#include <LoRaLib.h>

// create instance of LoRa class using SX1278 module
// this pinout corresponds to LoRenz shield: 
// https://github.com/jgromes/LoRenz
// NSS pin:   7 (18 on ESP32 boards)
// DIO0 pin:  2
// DIO1 pin:  3
// IMPORTANT: because this example uses external interrupts,
//            DIO0 MUST be connected to Arduino pin 2 or 3.
//            DIO1 MAY be connected to any free pin 
//            or left floating.
SX1278 lora = new LoRa;

void setup() {
  Serial.begin(9600);

  // initialize SX1278 with default settings
  Serial.print(F("Initializing ... "));
  // carrier frequency:           434.0 MHz
  // bandwidth:                   125.0 kHz
  // spreading factor:            9
  // coding rate:                 7
  // sync word:                   0x12
  // output power:                17 dBm
  // current limit:               100 mA
  // preamble length:             8 symbols
  // amplifier gain:              0 (automatic gain control)
  int state = lora.begin();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called 
  // when new packet is received
  lora.setDio0Action(setFlag);
  
  // start listening for LoRa packets
  Serial.print(F("Starting to listen ... "));
  state = lora.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // lora.standby()
  // lora.sleep()
  // lora.transmit();
  // lora.receive();
  // lora.scanChannel();
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;
    
    // you can read received data as an Arduino String
    String str;
    int state = lora.readData(str);
  
    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = lora.receive(byteArr, 8);
    */
    
    if (state == ERR_NONE) {
      // packet was successfully received
      Serial.println("Received packet!");
  
      // print data of the packet
      Serial.print("Data:\t\t\t");
      Serial.println(str);
  
      // print RSSI (Received Signal Strength Indicator) 
      Serial.print("RSSI:\t\t\t");
      Serial.print(lora.lastPacketRSSI);
      Serial.println(" dBm");
  
      // print SNR (Signal-to-Noise Ratio) 
      Serial.print("SNR:\t\t\t");
      Serial.print(lora.lastPacketSNR);
      Serial.println(" dBm");

      // print frequency error
      Serial.print("Frequency error:\t");
      Serial.print(lora.getFrequencyError());
      Serial.println(" Hz");
  
    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println("CRC error!");
  
    }

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }

}
