/*  ____  ____      _    __  __  ____ ___
   |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
   | | | | |_) |  / _ \ | |\/| | |  | | | |
   | |_| |  _ <  / ___ \| |  | | |__| |_| |
   |____/|_| \_\/_/   \_\_|  |_|\____\___/
                             research group
                               dramco.be/

    KU Leuven - Technology Campus Gent,
    Gebroeders De Smetstraat 1,
    B-9000 Gent, Belgium

           File: transmitter.ino
        Created: 2018-10-26
         Author: Chesney Buylle and Gilles Callebaut
        Version: 1.0
    Description:
              LoRa Transmitter for P2P communication

              Transmit LoRa packets with TRANSMISSION_INTERVAL milliseconds delays
              between them. Each packet contains 2 bytes
              of data, in the form of:
                - transmit power
                - spreading factor
*/

/*
 * 
 * the V2 version only transmits at SF12 TP 20 and transmits a counter
 */

#define NODE_UID 0xd0

#include <LoRaLibMod.h>

#define SPREADING_FACTOR               7

#define TRANSMISSION_INTERVAL         8000


#define TP                              20


#define DEBUG
#define DEBUG_ERR

// be sure that DEBUG ERR is defined if DEBUG is on
#ifdef DEBUG
#define DEBUG_ERR
#endif

#define POWER_ENABLE_PIN                8
#define LED                             4

float FREQ = 869.525; // %10 duty-cycle band
unsigned int counter = 0; // to hold counter which will be used to compute the PER


// create instance of LoRa class using SX1278 module
// NSS/CS pin:        6
// DIO0/INT0 pin:     2
// DIO1/INT1 pin:     3
SX1278 lora = new Module(6, 2, 3);


void setup() {
  Serial.begin(9600);

  pinMode(LED, OUTPUT);

  // Enable Pin 3V3 LDO to wake up LoRa module
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  delay(1000);
  // Initialize SX1278 with default settings
  debug(F("Initialization SX1278"));

  // carrier frequency:           434.0 MHz
  // bandwidth:                   125.0 kHz
  // spreading factor:            9
  // coding rate:                 7
  // sync word:                   0x12
  // output power:                17 dBm
  // current limit:               100 mA
  // preamble length:             8 symbols
  // amplifier gain:              0 (automatic gain control)

  // ------------------- INIT LoRa -------------------
  loraBegin();
  loraSetSF();
  loraSetTP();
  loraSetFreq();
}

void loop() {

  // ------------------- TX packet -------------------
  digitalWrite(LED, HIGH);
  sendPacket();
  delay(500);
  digitalWrite(LED, LOW);

  // ------------------- wait for new transmission -------------------
  uint32_t startTime = millis();
  while (millis() - startTime < TRANSMISSION_INTERVAL);
}

void sendPacket() {
  byte arr[3];
  arr[0] = (byte) NODE_UID;
  arr[1] = (byte) counter;
  arr[2] = (byte) counter >> 8;
  debug("Sending packet ... ");
  debug("with counter " + String(counter));

  int state = lora.transmit(arr, 3);

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    debug(" success!");
    counter++;

    // print measured data rate
    String s = "Datarate:\t";
    s.concat(lora.dataRate);
    s.concat(" bps");
    debug(s);

  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    error(F(" too long!"));

  } else if (state == ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    debug(F(" timeout!"));
  }

}


void blink() {
  while (1) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}


/* LORA specific methods */
void loraBegin(){
   int16_t state = lora.begin();
  if (state == ERR_NONE) {
    success(F("Initialization successful"));
  } else {
    error("Initialization failed", state);
  }
}



void loraSetSF(){
  // ------------------- SPREADING FACTOR -------------------
debug(F("Setting Spreading Factor"));
   int16_t state = lora.setSpreadingFactor(SPREADING_FACTOR);
  if (state != ERR_NONE) {
    error(state);
  }
  else {
    String s = "SF set to " + SPREADING_FACTOR;
    success(s);
  }
}
  
void loraSetTP(){
// ------------------- TRANSMIT POWER -------------------
 debug(F("Setting TP"));
  int16_t state = lora.setOutputPower(TP);
  if(state != ERR_NONE) {
    error(state);
  }
}

void loraSetFreq(){
// ------------------- CARRIER FREQ -------------------
  debug(F("Setting Frequency"));
  int16_t state = lora.setFrequency(FREQ);
  if (state != ERR_NONE) {
    error(state);
  }
  else {
    String s = "FREQ set ";
    s.concat(FREQ);
    success(s);
  }
}


/* SERIAL output information */
void debug(String s) {
#ifdef DEBUG
  Serial.println(s);
#endif
}

void error(String s) {
#ifdef DEBUG_ERR
  Serial.println(s);
#endif
  blink();
}

void error(String s, uint16_t state) {
#ifdef DEBUG_ERR
  Serial.println(s);
  Serial.println((int)state);
#endif
  blink();
}

void error(uint16_t state) {
#ifdef DEBUG_ERR
  Serial.println(state);
#endif
  blink();
}

void success(String s) {
#ifdef DEBUG
  Serial.println(s);
#endif
}
