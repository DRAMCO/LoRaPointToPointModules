// include the SD library:
#include <SPI.h>
#include "SdFat.h"

SdFat sd;
File file;

#define POWER_ENABLE_PIN  8
#define LED_PIN           A0
#define error(msg) sd.errorHalt(F(msg))

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
const int chipSelect = 4;
const uint8_t ANALOG_COUNT = 4;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("\nInitializing SD card...");
if (!sd.begin(chipSelect, SD_SCK_MHZ(10))) {
    Serial.println("initialization failed!");
    sd.initErrorHalt();
  }
  Serial.println("initialization done.");

  file = sd.open("test.txt",FILE_WRITE);
  if (!file) {
    error("file.open");
  }

  Serial.println("done!");

  logData();

  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  
}

void loop() {
  // nothing happens after setup finishes.
  
}

void logData() {
  uint16_t data[ANALOG_COUNT];

  // Read all channels to avoid SD write latency between readings.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    data[i] = analogRead(i);
  }
  // Write ADC data to CSV record.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.write(',');
    file.print(data[i]);
  }
  file.println();
}
