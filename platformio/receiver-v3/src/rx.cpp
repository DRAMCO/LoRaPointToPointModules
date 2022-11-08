//#define USE_GPS
//#define USE_SD
#define USE_SERIAl

#include <Arduino.h>
#include <SPI.h>

/*************** SD CARD ****************/
#ifdef USE_SD
#include "SdFat.h"
SdFat sd;
File file;
#define CHIP_SELECT 4
#define CAN_WE_WRITE_PIN A1

#define DELIMETER ','
#endif


/*************** GPS ****************/
#ifdef USE_GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define RXPin 7
#define TXPin 10
#define GPS_BAUD 9600

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element

uint32_t lastGPSupdate;
#define GPS_WRITE_UPDATE_INTERVAL 3000
#endif

/*************** LORA ****************/
#include <LoRaLibMod.h>
#define SPREADING_FACTOR 12
float FREQ = 869.525;
volatile bool receivedFlag = false;
SX1278 lora = new Module(6, 2, 3);
uint16_t counter = 0;
/*************** LORA ****************/

#define POWER_ENABLE_PIN 8
#define LED_PIN A0

static void smartDelay(unsigned long ms);

void blink(uint16_t interval, uint16_t durationOn)
{
  digitalWrite(LED_PIN, HIGH);
  smartDelay(durationOn);
  digitalWrite(LED_PIN, LOW);
  smartDelay(interval);
}


void blink()
{
  blink(200, 200);
}

void fastBlink(){
  blink(50,50);
}

void slowBlink()
{
  blink(2000,200);
}

#ifdef USE_SD
bool canWrite()
{
  while (analogRead(CAN_WE_WRITE_PIN) > 25)
  {
    if (file)
    {
      file.flush();
      file.close();
    }
    blink();
  }
  // reopen file after closing it due to button 
  if (!file)
  {
    file = sd.open(F("data.csv"), FILE_WRITE);
  }

  return true;
}
#endif

void error()
{
  #ifdef USE_SD
  if (file)
  {
    file.flush();
    file.close();
  }
  #endif
  while (1)
  {
    blink();
  }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    #ifdef USE_GPS
    while (ss.available())
      gps.encode(ss.read());
    #else
      delay(1);
    #endif
  } while (millis() - start < ms);
}

/* LORA specific methods */
void loraBegin()
{
  int16_t state = lora.begin();
  if (state == ERR_NONE)
  {
    Serial.println(F("Initialization successful"));
  }
  else
  {
    Serial.println("Initialization failed" + state);
    error();
  }
}

void loraListen()
{
  //  debug(F("Starting to listen ... "));
  int16_t state = lora.startReceive();
  if (state == ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.println("failed, code " + state);
    error();
  }
}

void loraSetSF(uint8_t sf)
{
  // ------------------- SPREADING FACTOR -------------------
  Serial.println(F("Setting Spreading Factor"));
  int state = lora.setSpreadingFactor(sf);
  if (state == ERR_NONE)
  {
    Serial.println(String(sf));
  }
  else
  {
    Serial.println("failed, code " + state);
    error();
  }
}

void loraSetFreq()
{
  // ------------------- CARRIER FREQ -------------------
  //debug(F("Setting Frequency"));
  int16_t state = lora.setFrequency(FREQ);
  if (state != ERR_NONE)
  {
    Serial.println(state);
    error();
  }
}

void receivePacket()
{
  byte arr[2];

  int state = lora.readData(arr, 2);
  if (state == ERR_NONE)
  {
    counter = uint16_t(arr[0]) | (arr[1] << 8);
    Serial.println(counter);
  }
  else
  {
    Serial.println("ERROR: " + state);
    error();
  }
}

// return if there was a packet received
bool checkRx()
{
  // Check if LoRa packet is received
  if (receivedFlag)
  {
    digitalWrite(LED_PIN, HIGH);
    receivePacket();
    #ifdef USE_SD
    writeToSD(true);
    #endif
    receivedFlag = false;
    digitalWrite(LED_PIN, LOW);
    return true;
  }
  return false;
}

void setMsgRx()
{
  receivedFlag = true;
}

void initLoRa()
{
  Serial.println(F("Initializing ... "));
  loraBegin();
  smartDelay(0);
  lora.setDio0Action(setMsgRx);
  smartDelay(0);
  loraSetSF(SPREADING_FACTOR);
  smartDelay(0);
  loraSetFreq();
  smartDelay(0);
  loraListen();
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

  smartDelay(0);
}
#endif

#ifdef USE_GPS
void initGPS(){
  Serial.println(F("Init GPS"));

  ss.begin(GPS_BAUD);
  smartDelay(2000);

  while (!gps.location.isValid())
  {
    smartDelay(2000);
    slowBlink();
    printDateTime(gps.date, gps.time);
    Serial.println(gps.satellites.value());
    Serial.println(gps.location.lat());
    Serial.println(gps.location.lng());

    Serial.println(gps.charsProcessed());
    Serial.println(gps.sentencesWithFix());
    Serial.println(gps.failedChecksum());
    Serial.println();
  }
  // 3 times fast blinking to indicate lock
  fastBlink();
  fastBlink();
  fastBlink();
}
#endif

#ifdef USE_SD
void initSDCard(){
  Serial.print("\nInitializing SD card...");
  if (!sd.begin(CHIP_SELECT, SD_SCK_MHZ(4)))
  {
    Serial.println("initialization failed!");
    error();
  }
  Serial.println(F("initialization done."));

  Serial.print(F("Opening test.txt file..."));
  file = sd.open(F("data.csv"), FILE_WRITE);
  if (!file)
  {
    error();
  }

  Serial.println("done!");
}
#endif

void initBoard(){
  Serial.println(F("Init Board"));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // do not proceed if button is not on
  #ifdef USE_SD
  canWrite();
  #endif

  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  #ifdef USE_SD
  initSDCard();
  #endif

  initLoRa();
  
  #ifdef USE_GPS
  initGPS();
  #endif

  smartDelay(0);
}

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

  smartDelay(0);
}
#endif

#ifdef USE_GPS
void writeToSD(bool packet)
{
  canWrite();

  Serial.println("writing...");

  if (file)
  {
    file.print(getDateTimeString(gps.date, gps.time));
    file.print(DELIMETER);
    file.print(gps.satellites.value(), 5);
    file.print(DELIMETER);
    file.print(gps.satellites.isValid());
    file.print(DELIMETER);
    file.print(gps.hdop.hdop(), 5);
    file.print(DELIMETER);
    file.print(gps.hdop.isValid());
    file.print(DELIMETER);
    file.print(gps.location.lat(), 6);
    file.print(DELIMETER);
    file.print(gps.location.lng(), 6);
    file.print(DELIMETER);
    file.print(gps.location.isValid());
    file.print(DELIMETER);
    file.print(gps.location.age(), 5);
    file.print(DELIMETER);
    file.print(gps.location.isValid());
    file.print(DELIMETER);
    file.print(gps.altitude.meters(), 6);
    file.print(DELIMETER);
    file.print(gps.altitude.isValid());
    file.print(DELIMETER);
    file.print(lora.lastPacketRSSI);
    file.print(DELIMETER);
    file.print(lora.lastPacketSNR);
    file.print(DELIMETER);
    file.print(counter);

    file.print(DELIMETER);
    file.println(packet);

    file.flush();

    //debug(F("DONE."));
  }else{
    error();
  }
}
#endif

void setup()
{

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  initBoard();

  #ifdef USE_GPS
  lastGPSupdate = millis();
  #endif
}

void loop()
{
  smartDelay(0);

  checkRx();

  #ifdef USE_SD
  canWrite();
  #endif

  #ifdef USE_GPS
  if (millis() - lastGPSupdate > GPS_WRITE_UPDATE_INTERVAL)
  {
    #ifdef USE_SD
    writeToSD(false);
    #endif
    lastGPSupdate = millis();
  }
  #endif

}
