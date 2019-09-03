// include the SD library:
#include <SPI.h>
#include "SdFat.h"


#include <TinyGPS++.h>
#include <SoftwareSerial.h>


#include <LoRaLibMod.h>
#define SPREADING_FACTOR          12
float FREQ = 869.525;
volatile bool receivedFlag = false;
SX1278 lora = new Module(6, 2, 3);

unsigned int counter =0;

SdFat sd;
File file, readFile;

#define POWER_ENABLE_PIN  8
#define LED_PIN           A0

#define error(msg) sd.errorHalt(F(msg))

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
const int chipSelect = 4;


// GPS
static const int RXPin = 7, TXPin = 10;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void testBuzzer() {
  //Serial.println("Testing buzzer en LED");

  Serial.println(F("************************* TESTING BUZZER *************************"));


  for (int i = 0; i < 10; i++) {
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }

  Serial.println(F("*********************** DONE TESTING BUZZER ***********************"));
  Serial.println();
}

void testSDCard() {
  Serial.println(F("************************* TESTING SD CARD *************************"));
  Serial.print("\nInitializing SD card...");
  if (!sd.begin(chipSelect, SD_SCK_MHZ(4))) {
    Serial.println("initialization failed!");
    sd.initErrorHalt();
  }
  Serial.println(F("initialization done."));



  Serial.print(F("Opening test.txt file..."));
  file = sd.open(F("test.txt"), FILE_WRITE);
  if (!file) {
    error("file.open");
  }

  Serial.println("done!");

  Serial.print("Removing test.txt...");
  if (!sd.remove("test.txt")) {
    error("file.remove");
  }
  Serial.println("done!");

  Serial.print("Re-opening test.txt file...");
  file = sd.open("test.txt", FILE_WRITE);
  if (!file) {
    error("file.open");
  }

  Serial.println("done!");

  Serial.print(F("Writing to test.txt file..."));
  file.println(F("testing 1, 2, 3."));
  Serial.println(F("done!"));

  Serial.print(F("Syncing and closing test.txt file..."));
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }
  file.close();
  Serial.println(F("done!"));

  // re-open the file for reading:
  Serial.print(F("Re-opening test.txt file..."));
  readFile = sd.open(F("test.txt"));
  if (readFile) {
    Serial.println(F("test.txt:"));

    // read from the file until there's nothing else in it:
    while (readFile.available()) {
      Serial.write(readFile.read());
    }
    // close the file:
    readFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }

  Serial.println(F("*********************** DONE TESTING SD CARD ***********************"));
  Serial.println();

}

void testGPS() {
  ss.begin(GPSBaud);

  bool stop = false;
  while (!stop) {
    if (!gps.location.isValid() && millis() > 10000) {
        Serial.println(F("No GPS data received: check wiring"));
      }
    if(gps.location.isValid() ) {
        Serial.println("GPS check!");
        stop= true;
      }

      
    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);


    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);
    Serial.println();

    smartDelay(1000);

   
      
    }


  }
  

  // This custom version of delay() ensures that the gps object
  // is being "fed".
  static void smartDelay(unsigned long ms)
  {
    unsigned long start = millis();
    do
    {
      while (ss.available())
        gps.encode(ss.read());
    } while (millis() - start < ms);
  }


  void setup() {
    //Enable Pin 3V3 LDO
    Serial.println(F("Enaling LDO ... "));
    pinMode(POWER_ENABLE_PIN, OUTPUT);
    digitalWrite(POWER_ENABLE_PIN, HIGH);
    
    // Open serial communications and wait for port to open:
    Serial.begin(9600);

    // disable LED and buzzer
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }

    

    testBuzzer();

    testSDCard();

    testGPS();

    testLoRa();

  }

  void loop() {
    // nothing happens after setup finishes.

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

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}


void testLoRa(){

  
  
  initLoRa();

  bool stop= false;

  Serial.println(F("Listening"));

  while(!checkRx()){}
  
}

// return if there was a packet received
bool checkRx() {
  // Check if LoRa packet is received
  if (receivedFlag) {
    digitalWrite(LED_PIN, HIGH);
    receivePacket();
    receivedFlag = false;
    digitalWrite(LED_PIN, LOW);
    return true;
  }
  return false;
}


bool receivePacket() {
  byte arr[2];

  int state = lora.readData(arr, 2);
  if (state == ERR_NONE) {
    //debug(F("PACKET RECEIVED"));
    counter = uint16_t (arr[0]) | (arr[1] << 8);
    Serial.println(counter);
  }else{
    Serial.println("ERROR: " + state);
  }

}


void setMsgRx() {
  receivedFlag = true;
}

void initLoRa() {
  Serial.println(F("Initializing ... "));
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
    Serial.println(F("Initialization successful"));
  } else {
    Serial.println("Initialization failed"+ state);
  }
}

void loraListen() {
  //  debug(F("Starting to listen ... "));
  int16_t state = lora.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.println("failed, code " + state);
  }
}


void loraSetSF(uint8_t sf) {
  // ------------------- SPREADING FACTOR -------------------
  Serial.println(F("Setting Spreading Factor"));
  int state = lora.setSpreadingFactor(sf);
  if (state != ERR_NONE) {
    Serial.println(state);
  }
  else {
    Serial.println(String(sf));
  }
}


void loraSetFreq() {
  // ------------------- CARRIER FREQ -------------------
  //debug(F("Setting Frequency"));
  int16_t state = lora.setFrequency(FREQ);
  if (state != ERR_NONE) {
    Serial.println(state);
  }
  else {
    //String s = "FREQ set ";
    //s.concat(FREQ);
    //success(s);
  }
}
