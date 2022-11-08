#include "LoraPTPApplication.h"
#include "Dramco-UNO-Sensors.h"
#include "CircBuffer.h"

// Singleton instance of the radio driver
RH_RF95 rf95(PIN_MODEM_SS, PIN_MODEM_INT);

MsgReceivedCb mrc = NULL;

/*--------------- UTILS ----------------- */
static void wdtYield(){
    DramcoUno.loop();
}

void printBuffer(uint8_t * buf, uint8_t len, bool newLine){
#pragma region DEBUG
#ifdef DEBUG
    for(uint8_t i; i<len; i++){
        if(buf[i]<16) Serial.print(" 0");
        else Serial.print(' ');
        Serial.print(buf[i], HEX);
    }
    if(newLine)
        Serial.println();
#endif
#pragma endregion
}

/*--------------- CONSTRUCTOR ----------------- */
LoRaPTP::LoRaPTP(){
    
}

/*--------------- BEGIN ----------------- */
bool LoRaPTP::begin(){
    // Dramco uno - enable 3v3 voltage regulator
    pinMode(PIN_ENABLE_3V3, OUTPUT);
    digitalWrite(PIN_ENABLE_3V3, HIGH);

#pragma region DEBUG
#ifdef DEBUG
    Serial.println(F("done."));
    Serial.print(F("Initializing modem... "));
#endif
#pragma endregion

    if(!rf95.init()){
#pragma region DEBUG
#ifdef DEBUG
        Serial.println(F("failed."));
#endif
#pragma endregion
        return false;
    }

    this->reconfigModem();

#pragma region DEBUG
#ifdef DEBUG
    Serial.println(F("done"));
    Serial.print(F("Setting node UID: "));
#endif
#pragma endregion

    
    this->uid = NODE_UID;

    uint32_t r = rf95.random() * DramcoUno.random();
    randomSeed(r);  

#pragma region DEBUG
#ifdef DEBUG
    Serial.println(uid, HEX);
#endif
#pragma endregion
    
    return true;
}

/*--------------- LOOP ----------------- */
void LoRaPTP::loop(void){

    /*rf95.setModeCad(); // listen for channel activity
    
    if(!this->waitCADDone()){
#pragma region DEBUG
#ifdef DEBUG
        Serial.println(F("CAD failed"));
#endif
#pragma endregion
    };*/

    // if channel activity has been detected during the previous CAD - enable RX and receive message
    if(true){
        rf95.setModeRx();
        rf95.waitAvailableTimeout(random(3*PREAMBLE_DURATION,6*PREAMBLE_DURATION));
        uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
        if(rf95.recv(this->rxBuf, &len)){
#pragma region DEBUG
#ifdef DEBUG
            Serial.println(F("Message received."));
#endif
#pragma endregion
            if(this->handleAnyRxMessage(this->rxBuf, len)){
                if(mrc != NULL){
                    //mrc(this->rxBuf+HEADER_NODE_UID_OFFSET, len-HEADER_NODE_UID_OFFSET);
                    mrc(this->rxBuf, len);
                }
            }
        }
        else{
#ifdef DEBUG
            //Serial.println(F("Message receive failed"));
#endif
        }
        // Reschedule TX and PresetSend to allow for backoff 
        // This is TX: just for breacon and broadcast
        // TODO: make this clear, can this be just to the next CAD? 
        unsigned long now = DramcoUno.millisWithOffset();
        if(this->txPending && (now > this->txTime-COLLISION_DELAY_MAX)){
            this->txTime = now + random(COLLISION_DELAY_MIN,COLLISION_DELAY_MAX); // If CAD detected, add 150-300ms to pending schedule time of next message
        }
    }
    else{
        // handle any pending tx (beacon or broadcast)
        unsigned long now = DramcoUno.millisWithOffset();
        if(this->txPending && (now > this->txTime)){
            this->txMessage(txLen);
            this->txTime = now + random(COLLISION_DELAY_MIN,COLLISION_DELAY_MAX);
        }
    }
    
    DramcoUno.loop();

}

/*--------------- PHYSICAL RELATED METHODS ----------------- */

bool LoRaPTP::waitCADDone( void ){
    DramcoUno.fastSleep();
    return rf95.cadDone();
}

bool LoRaPTP::waitRXAvailable(uint16_t timeout){
    unsigned long starttime = DramcoUno.millisWithOffset();
    while ((DramcoUno.millisWithOffset() - starttime) < timeout){
        DramcoUno.fastSleep();
        if (rf95.available()){
           return true;
	}
	YIELD;
    }
    return false;
}

void LoRaPTP::reconfigModem(void){
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    rf95.setFrequency(868);

    ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
    rf95.setPreambleLength(PREAMBLE_DURATION*390/100); // 100 ms cycle for wakeup
    // From my understanding of the datasheet, preamble length in symbols = cycletime * BW / 2^SF. So,
    // for 100ms  cycle at 500 kHz and SF7, preamble = 0.100 * 500000 / 128 = 390.6 symbols.
    // Max value 65535

    // lowest transmission power possible
    rf95.setTxPower(5, false);
}

void LoRaPTP::setMsgReceivedCb(MsgReceivedCb cb){
    mrc = cb;
}

void LoRaPTP::txMessage(uint8_t len){
    rf95.send(this->txBuf, len);
    void (*  fptr)() = &wdtYield;
    rf95.waitPacketSent(1000, fptr);
    this->txPending = false;

#pragma region DEBUG
#ifdef DEBUG
    Serial.print(F("TX MSG: "));
    printBuffer(this->txBuf, len, true);
#endif
#pragma endregion

}

/*--------------- PROTOCOL RELATED METHODS ----------------- */
/*--- 1. Message type independant methods ---*/
bool LoRaPTP::handleAnyRxMessage(uint8_t * buf, uint8_t len){
    if(len < HEADER_SIZE){
        return false;
    }

#pragma region DEBUG
#ifdef DEBUG
    Serial.print(F("RX MSG: "));
    printBuffer(buf, len, false);
    Serial.print(F(", RSSI: "));
    rf95.lastRssi();
    Serial.print(F(", SNR: "));
    rf95.lastSnr();
#endif
#pragma endregion
    
    return false;
}

bool LoRaPTP::sendMessage(uint8_t * payload, uint8_t len){
    // Generate message uid
    Msg_UID_t mUID = (uint16_t) random();

    // Build header
    this->setFieldInBuffer(mUID, this->txBuf, HEADER_MESG_UID_OFFSET, sizeof(Msg_UID_t));
    this->setFieldInBuffer(this->uid, this->txBuf, HEADER_SIZE, sizeof(Node_UID_t));
    uint8_t pos = PAYLOAD_DATA_OFFSET+HEADER_SIZE;
    memcpy(this->txBuf+PAYLOAD_DATA_OFFSET+HEADER_SIZE, payload, len);

    // Final transmit message (NOW)
    this->txMessage(len+HEADER_SIZE+sizeof(Node_UID_t));
    this->txPending = true;
    this->txTime = DramcoUno.millisWithOffset() + random(COLLISION_DELAY_MIN,COLLISION_DELAY_MAX); // If CAD detected, add 150-300ms to pending schedule time of next message
    return true;
}

bool LoRaPTP::setFieldInBuffer(BaseType_t field, uint8_t * buf, uint8_t fieldOffset, size_t size){
    if(size == 0 || size > sizeof(BaseType_t)){
        return false;
    }

    // TODO: check buf length?

    BaseType_t temp = field;
    for(int8_t index=fieldOffset+size-1; index>=fieldOffset; index--){
        buf[index] = temp & 0xFF;
        temp >>= 8;
    }
    return true;
}
