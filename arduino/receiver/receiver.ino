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

              Receive LoRa packets

              Each packet contains 2 bytes
              of data, in the form of:
                - transmit power
                - spreading factor

                LoRa Receiver for P2P communication
                This example listens for LoRa transmissions and tries to
                receive them. To successfully receive data, the following
                settings have to be the same on both transmitter
                and receiver:
                 - carrier frequency
                 - bandwidth
                 - spreading factor
                 - coding rate
                 - sync word
                 - preamble length
*/


#include <LoRaLibMod.h>
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

// uncomment if you want to listen to one specific SF (7, 9, 12)
#define ONLY_SF                   12

#define POWER_ENABLE_PIN          8
#define GPS_RX_PIN                7
#define GPS_TX_PIN                10
#define CAN_WE_WRITE_PIN          A1
#define SD_CS                     4

#define GPS_WRITE_UPDATE_INTERVAL 1000
#define MAX_TRANSMISSIONCASE      3
#define DELIMETER                 ','
#define GPS_BAUD                  9600
#define MAX_SETTINGS              3
#define RANDOM_PIN                A3
#define LED_PIN                   A0


float FREQ = 869.525;

#define DEBUG
//#define DEBUG_ERR

// be sure that DEBUG ERR is defined if DEBUG is on
#ifdef DEBUG
#define DEBUG_ERR
#endif


// create instance of LoRa class using SX1278 module
// NSS/CS pin:        6
// DIO0/INT0 pin:     2
// DIO1/INT1 pin:     3
SX1278 lora = new Module(6, 2, 3);

File myFile;

const uint8_t SPREADING_FACTORS[] = {7, 9, 12};
const uint16_t WINDOW_TIME_PER_SF[] = {15000, 20000, 30000}; // * X interval for each SF
uint8_t current_spreading_factor_id = 0;

// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);

TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element

float latitude;
float longitude;
boolean locationValid = false;
unsigned long satVal;
boolean satValid = false;
float hdopVal;
float vdopVal;
float pdopVal;
boolean hdopValid = false;
unsigned long ageVal;
boolean ageValid = false;
float altitudeVal;
boolean altitudeValid = false;
uint8_t spreadingFactor = 0;

boolean packetReceived = false;

uint32_t lastGPSupdate;
uint32_t lastSettingsChanged;
uint32_t lastPacketReceived;

volatile bool receivedFlag = false;

bool hasFix = false;
String fileName;


void blink() {
  //Serial.println("in blink");
  if (myFile) {
    myFile.close();
  }
  while (analogRead(CAN_WE_WRITE_PIN) > 25) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  myFile = SD.open(fileName, FILE_WRITE);
  if (!myFile) {
    Serial.println("File error");
    blink();
  }
}



void gpsFix() {
  //Serial.println("in blink");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}


void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  randomSeed(analogRead(RANDOM_PIN));
  checkCanWrite();

#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Begin serial connection to the GPS device
  ss.begin(GPS_BAUD);

  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  // Initialize RFM95 LoRa module
  initLoRa();

  checkCanWrite();
#ifdef DEBUG
  debug(F("Initializing SD card..."));
#endif
  if (!SD.begin(SD_CS)) {
#ifdef DEBUG
    Serial.println(F("initialization SD failed!"));
#endif
    blink();
  } else {
    fileName = getRandomFileName();
    Serial.println(fileName);
    myFile = SD.open(fileName, FILE_WRITE);
    if (!myFile) {
      Serial.println("File error");
      blink();
    }
  }
  //debug(F("initialization SD done."));

  lastGPSupdate = millis();
  lastSettingsChanged = millis();
}

String getRandomFileName() {
  // 8.3 file format!
  // generate string based on ASCII
  // 48 '0' till 57 '9'
  // 65 'A' till 90 'Z'
  // 97 'a' till 122 'z'
  String s = "";
  for (int i = 0; i < 8; i++) {
    byte r = random(0, 3);
    char letter = '0';
    if (r == 0) letter = (char) random(48, 58);
    else if (r == 1) letter = (char) random(65, 91);
    else if (r == 2) letter = (char) random(97, 123);
    s.concat(letter);
  }
  s.concat(".csv");
  return s;
}



String getDateTimeString(TinyGPSDate &d, TinyGPSTime &t)
{
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());

  String s = sz;


  sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
  s.concat(sz);
  //debug(s);
  return s;

  smartDelay(0);
}

void writeToSD(bool packet) {
  checkCanWrite();

  Serial.println("writing...");

  if (!hasFix && gps.location.isValid()) {
      gpsFix();
      hasFix = true;
    }
  


  if (myFile) {
    myFile.print(getDateTimeString(gps.date, gps.time));
    myFile.print(DELIMETER);
    myFile.print(satVal, 5);
    myFile.print(DELIMETER);
    myFile.print(satValid);
    myFile.print(DELIMETER);
    myFile.print(hdopVal, 5);
    myFile.print(DELIMETER);
    myFile.print(hdopValid);
    myFile.print(DELIMETER);
    myFile.print(vdopVal, 5);
    myFile.print(DELIMETER);
    myFile.print(pdopVal, 5);
    myFile.print(DELIMETER);
    myFile.print(latitude, 6);
    myFile.print(DELIMETER);
    myFile.print(longitude, 6);
    myFile.print(DELIMETER);
    myFile.print(locationValid);
    myFile.print(DELIMETER);
    myFile.print(ageVal, 5);
    myFile.print(DELIMETER);
    myFile.print(ageValid);
    myFile.print(DELIMETER);
    myFile.print(altitudeVal, 6);
    myFile.print(DELIMETER);
    myFile.print(altitudeValid);
    myFile.print(DELIMETER);
    myFile.print(lora.lastPacketRSSI);
    myFile.print(DELIMETER);
    myFile.print(lora.lastPacketSNR);
    myFile.print(DELIMETER);
    myFile.print(spreadingFactor);

    myFile.print(DELIMETER);
    myFile.println(packet);

    //debug(F("DONE."));
  }
}

void loop() {
  checkRx();

  uint32_t currentTime = millis();

  if (analogRead(CAN_WE_WRITE_PIN) > 25) {
    blink();
  }

  if (currentTime - lastGPSupdate > GPS_WRITE_UPDATE_INTERVAL) {
    writeToSD(false);
    lastGPSupdate = millis();
  }

  checkRx();

  if (currentTime - lastSettingsChanged > WINDOW_TIME_PER_SF[current_spreading_factor_id]) {
    // if lastPacketReceived inside rx window do not change SF
    if (currentTime - lastPacketReceived > WINDOW_TIME_PER_SF[current_spreading_factor_id]) {
      hopToDifferentSF();
      lastSettingsChanged = millis();
    }
    myFile.flush();
  }

  checkRx();

  receiveGPS();

  checkRx();


}

void checkRx() {
  // Check if LoRa packet is received
  if (receivedFlag) {
    digitalWrite(LED_PIN, HIGH);
    receivePacket();
    receivedFlag = false;
    digitalWrite(LED_PIN, LOW);
  }
}

void hopToDifferentSF() {
#ifdef ONLY_SF
#if ONLY_SF == 7
  if (current_spreading_factor_id != 0) current_spreading_factor_id = 0;
  else return;
#elif ONLY_SF == 9
  if (current_spreading_factor_id != 1) current_spreading_factor_id = 1;
  else return;
#elif ONLY_SF == 12
  if (current_spreading_factor_id != 2) current_spreading_factor_id = 2;
  else return;
#endif
#else
  current_spreading_factor_id = (current_spreading_factor_id + 1) % MAX_SETTINGS;
#endif
  loraSetSF(SPREADING_FACTORS[current_spreading_factor_id]);
  loraListen();
}


bool receivePacket() {
  byte arr[1];

  int state = lora.readData(arr, 1);
  if (state == ERR_NONE) {
    //debug(F("PACKET RECEIVED"));
    spreadingFactor = arr[0];
    //debug(String(spreadingFactor));
    lastPacketReceived = millis();
  } else {
    //error(F("packet rx?"), state);
  }

  writeToSD(true);

}

// This custom version of delay() ensures that the gps object is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms && !receivedFlag);
}

void receiveGPS() {
  smartDelay(1000);

  latitude = gps.location.lat();
  longitude = gps.location.lng();
  locationValid = gps.location.isValid();
  satVal = gps.satellites.value();
  satValid = gps.satellites.isValid();
  hdopVal = gps.hdop.hdop();
  hdopValid = gps.hdop.isValid();
  ageVal = gps.location.age();
  ageValid = gps.location.isValid();
  altitudeVal = gps.altitude.meters();
  altitudeValid = gps.altitude.isValid();
  pdopVal = String(pdop.value()).toFloat();
  vdopVal = String(vdop.value()).toFloat();

}

void checkCanWrite() {
  if (analogRead(CAN_WE_WRITE_PIN) > 25) {
    blink();
  }
}

void setMsgRx() {
  receivedFlag = true;
}

void initLoRa() {
  //debug(F("Initializing ... "));
  loraBegin();
  lora.setDio0Action(setMsgRx);
  loraSetSF(SPREADING_FACTORS[current_spreading_factor_id]);
  loraSetFreq();
  loraListen();
}

/* LORA specific methods */
void loraBegin() {
  int16_t state = lora.begin();
  if (state == ERR_NONE) {
    //success(F("Initialization successful"));
  } else {
    error("Initialization failed", state);
  }
}

void loraListen() {
  //  debug(F("Starting to listen ... "));
  int16_t state = lora.startReceive();
  if (state == ERR_NONE) {
    //success(F("success!"));
  } else {
    error(F("failed, code "), state);
  }
}


void loraSetSF(uint8_t sf) {
  // ------------------- SPREADING FACTOR -------------------
  //debug(F("Setting Spreading Factor"));
  int state = lora.setSpreadingFactor(sf);
  if (state != ERR_NONE) {
    error(state);
  }
  else {
    //success(String(sf));
  }
}


void loraSetFreq() {
  // ------------------- CARRIER FREQ -------------------
  //debug(F("Setting Frequency"));
  int16_t state = lora.setFrequency(FREQ);
  if (state != ERR_NONE) {
    error(state);
  }
  else {
    //String s = "FREQ set ";
    //s.concat(FREQ);
    //success(s);
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
  Serial.println(state);
#endif
  blink();
}

void error(uint16_t state) {
#ifdef DEBUG_ERR
  Serial.println(state);
#endif
  blink();
}

/*
  void success(String s) {
  #ifdef DEBUG
  Serial.println(s);
  #endif

  }
*/
