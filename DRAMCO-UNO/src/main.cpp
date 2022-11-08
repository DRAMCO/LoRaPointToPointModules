// Example for flooding mesh network using CAD
// Extra low power: disable 3v3 regulator on Dramco Uno => EXTRA CAPS NEEDED ON VCC RF95 (100nF // 10nF)

#include <Arduino.h>
#include "LoRaPTPApplication.h"

#include "Dramco-UNO-Sensors.h"

#define PIN_BUTTON        10
#define PIN_LED           4

#define DRAMCO_UNO_LPP_DIGITAL_INPUT_MULT   1
#define DRAMCO_UNO_LPP_ANALOG_INPUT_MULT    100
#define DRAMCO_UNO_LPP_GENERIC_SENSOR_MULT  1
#define DRAMCO_UNO_LPP_LUMINOSITY_MULT      1
#define DRAMCO_UNO_LPP_TEMPERATURE_MULT     10
#define DRAMCO_UNO_LPP_ACCELEROMETER_MULT   1000
#define DRAMCO_UNO_LPP_PERCENTAGE_MULT      1


bool newMsg = false;
bool measureNow = false;

uint8_t payloadBuf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t payloadLen = 0;

uint16_t ctr = 0x00;

unsigned long prevMeasurement = 0;

LoRaPTP ptp;


// callback function for when a new message is received
void msgReceived(uint8_t * packet, uint8_t plen){
#ifdef DEBUG
  //Serial.begin(115200);
  Serial.print("Packet: ");
#endif
  for(uint8_t i=0; i<plen; i++){
#ifdef DEBUG
    if(packet[i] < 16){
        Serial.print('0');
    }
    Serial.print(packet[i], HEX);
    Serial.print(' ');
#endif
    payloadBuf[i] = packet[i];
  }
#ifdef DEBUG
  Serial.println();
#endif

  payloadLen = plen;
  newMsg = true;
}


void setup(){
  // Start serial connection for printing debug information
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(F("LoRa PTP multihop demo."));
#endif

#ifdef DEBUG
  Serial.println(F("Starting Dramco Uno firmware..."));
#endif
  DramcoUno.begin();

Serial.println("Random number:");
Serial.println(DramcoUno.random());
#ifdef DEBUG
  Serial.println(F("Starting lora multihop..."));
#endif
  if(!ptp.begin()){
    while(true){
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(100);
    }
  }
  ptp.setMsgReceivedCb(&msgReceived);
#ifdef DEBUG
  Serial.println(F("done."));
#endif

#ifdef DEBUG
  Serial.println(F("Setting button press interrupt."));
#endif
DramcoUno.interruptOnButtonPress();

#ifdef DEBUG
  Serial.println(F("Setup complete."));
  Serial.println(F("Press the button to send a message."));
#endif
}

void loop(){
  ptp.loop();

  if((DramcoUno.millisWithOffset() - prevMeasurement) > SEND_INTERVAL){
    prevMeasurement = DramcoUno.millisWithOffset();
    measureNow = true;
  }

  // button press initiates a "send message"
  if(DramcoUno.processInterrupt() || measureNow){
    measureNow = false;

#ifdef DEBUG
    Serial.print(F("Own ID: "));
    Serial.println(NODE_UID, HEX);
    Serial.println(F("Composing message"));
    
#endif

    uint8_t data[15];
    uint8_t i = 0;
    uint16_t vt = ctr++; // use counter instead
    
    data[i++] = (uint8_t)(vt >> 8);
    data[i++] = (uint8_t)vt;
    ptp.sendMessage(data, i);
    
    DramcoUno.interruptOnButtonPress();
  }

  if(newMsg){
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    newMsg = false;
  }
  digitalWrite(DRAMCO_UNO_LED_NAME, LOW);
}
