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

//#define USE_GPS
//#define USE_SD

#define NODE_UID 0xB0

#include <LoRaLibMod.h>
#include <SPI.h>

#ifdef USE_GPS
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>
#endif

#ifdef USE_SD
#include <SD.h>
#endif


// uncomment if you want to listen to one specific SF (7, 9, 12)
// --------------- LORA ------------------
#define SPREADING_FACTOR          7
float FREQ = 869.525;
#define BW 125
#define TRANSMISSION_INTERVAL         8000

// --------------- HAL -------------------
#define POWER_ENABLE_PIN          8
#define GPS_RX_PIN                7
#define GPS_TX_PIN                10
#define CAN_WE_WRITE_PIN          A1
#define SD_CS                     4
#define RANDOM_PIN                A3
#define LED_PIN                   A0

// -------------- GENERAL ---------------

#define MAX_TRANSMISSIONCASE      3
#define MAX_SETTINGS              3


// -------------- SD ---------------
#define DELIMETER                 ','

// -------------- GPS ---------------
#define GPS_BAUD                  9600
#define GPS_WRITE_UPDATE_INTERVAL 1000
#define FLUSH_INTERVAL            20000



#define DEBUG
#define DEBUG_ERR

// be sure that DEBUG ERR is defined if DEBUG is on
#ifdef DEBUG
#define DEBUG_ERR
#endif


// create instance of LoRa class using SX1278 module
// NSS/CS pin:        6
// DIO0/INT0 pin:     2
// DIO1/INT1 pin:     3
SX1278 lora = new Module(6, 2, 3);

#ifdef USE_SD
File myFile;
#endif

#ifdef USE_GPS
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
#endif

unsigned int counterRx =0;

unsigned int counterTx =0;

boolean packetReceived = false;

#ifdef USE_GPS
uint32_t lastGPSupdate;
#endif
#ifdef USE_SD
uint32_t lastSDFlushed;
#endif
uint32_t lastPacketReceived;


volatile bool receivedFlag = false;
#ifdef USE_GPS
bool hasFix = false;
#endif
#ifdef USE_SD
String fileName;
#endif

uint32_t startTime = 0;

void blink() {
  //Serial.println("in blink");
  #ifdef USE_SD
  if (myFile) {
    myFile.close();
  }
  
  while (analogRead(CAN_WE_WRITE_PIN) > 25) {
  #endif
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  #ifdef USE_SD
  }
  myFile = SD.open(fileName, FILE_WRITE);
  if (!myFile) {
    Serial.println("File error");
    blink();
  }
  #endif
}


#ifdef USE_GPS
void gpsFix() {
  //Serial.println("in blink");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}
#endif

#ifdef USE_GPS
void initGPS(){
  // Begin serial connection to the GPS device
  ss.begin(GPS_BAUD);

  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}
#endif


void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  randomSeed(analogRead(RANDOM_PIN));
  #ifdef USE_SD
  checkCanWrite();
  #endif
  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(POWER_ENABLE_PIN, HIGH);
  delay(1000);
#ifdef DEBUG
  Serial.begin(115200);
#endif

  #ifdef USE_GPS
  initGPS();
  #endif
  

  // Initialize RFM95 LoRa module
  initLoRa();
  
  #ifdef USE_SD
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
  #endif
  //debug(F("initialization SD done."));

  #ifdef USE_GPS
  lastGPSupdate = millis();
  lastSDFlushed = millis();
  #endif
  smartDelay(10);

  startTime = millis();
}

#ifdef USE_SD
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
#endif


#ifdef USE_GPS
String getDateTimeString(TinyGPSDate &d, TinyGPSTime &t)
{
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());

  String s = sz;


  sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
  s.concat(sz);
  //debug(s);
  return s;

  smartDelay(10);
}
#endif

#ifdef USE_SD
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
    myFile.print(counterRx);

    myFile.print(DELIMETER);
    myFile.println(packet);

    debug(getDateTimeString(gps.date, gps.time));

    //debug(F("DONE."));
  }
}
#endif

void loop() {
  smartDelay(10);
  checkRx();

  // uint32_t currentTime = millis();

  #ifdef USE_SD
  if (analogRead(CAN_WE_WRITE_PIN) > 25) {
    blink();
  }
  #endif

  #ifdef USE_GPS
  if (currentTime - lastGPSupdate > GPS_WRITE_UPDATE_INTERVAL) {
    writeToSD(false);
    lastGPSupdate = millis();
  }
  #endif

  // checkRx();
  // smartDelay(10);

  #ifdef USE_SD
  if (currentTime - lastSDFlushed > FLUSH_INTERVAL) {
    myFile.flush();
    lastSDFlushed = millis();
  }
  #endif

  // checkRx();

  #ifdef USE_GPS
  receiveGPS();
  #endif
  
  // checkRx();

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



bool receivePacket() {
  byte arr[3];

  int state = lora.readData(arr, 3);
  if (state == ERR_NONE) {
    counterRx = uint16_t (arr[1]) | (arr[2] << 8);
    Serial.print(arr[0], HEX);
    Serial.print(',');
    Serial.print(arr[2], HEX);
    Serial.print(arr[1], HEX);
    Serial.print(',');
    Serial.print(SPREADING_FACTOR);
    Serial.print(',');
    Serial.print(BW);
    Serial.print(',');
    Serial.print(lora.lastPacketRSSI);
    Serial.print(',');
    Serial.println(lora.lastPacketSNR);
    lastPacketReceived = millis();
  } else {
    //error(F("packet rx?"), state);
  }
  #ifdef USE_SD
  writeToSD(true);
  #endif

}


#ifdef USE_GPS
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
#endif
#ifdef USE_SD
void checkCanWrite() {
  if (analogRead(CAN_WE_WRITE_PIN) > 25) {
    blink();
  }
}
#endif

void setMsgRx() {
  receivedFlag = true;
}

void initLoRa() {
  debug(F("Initializing ... "));
  loraBegin();
  lora.setDio0Action(setMsgRx);
  loraSetSF(SPREADING_FACTOR);
  loraSetFreq();
  loraListen();
}

/* LORA specific methods */
void loraBegin() {
  int16_t state = lora.begin();
  if (state == ERR_NONE) {
    success(F("Initialization successful"));
  } else {
    error("Initialization failed", state);
  }
}

void loraListen() {
  Serial.println(F("Starting to listen ... "));
  int16_t state = lora.startReceive();
  if (state == ERR_NONE) {
    success(F("success!"));
  } else {
    error(F("failed, code "), state);
  }
}


void loraSetSF(uint8_t sf) {
  // ------------------- SPREADING FACTOR -------------------
  debug(F("Setting Spreading Factor"));
  int state = lora.setSpreadingFactor(sf);
  if (state != ERR_NONE) {
    error(state);
  }
  else {
    success(String(sf));
  }
}

void loraSetFreq() {
  // ------------------- CARRIER FREQ -------------------
  //debug(F("Setting Frequency"));
  int16_t state = lora.setFrequency(FREQ);
  state = lora.setRxBandwidth(BW);
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


  void success(String s) {
  #ifdef DEBUG
  Serial.println(s);
  #endif

  }


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  #ifdef USE_GPS
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
  #else
  delay(ms);
  #endif
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}
#ifdef USE_GPS
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}
#endif

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
