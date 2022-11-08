#ifndef __LORA_PTP__
#define	__LORA_PTP__

#include <Arduino.h>
#include <RH_RF95.h>
#include "CircBuffer.h"


#define PREAMBLE_DURATION                   12.54 // In ms

#define COLLISION_DELAY                     2*PREAMBLE_DURATION       // Backoff if CAD detected when wanting to send
#define COLLISION_DELAY_RANDOM              PREAMBLE_DURATION
#define COLLISION_DELAY_MIN                 COLLISION_DELAY-COLLISION_DELAY_RANDOM/2       // Backoff if CAD detected when wanting to send
#define COLLISION_DELAY_MAX                 COLLISION_DELAY-COLLISION_DELAY_RANDOM/2

#define TX_BUFFER_SIZE                      100     // Max preset buffer size 
#define PAYLOAD_TX_THRESHOLD                64

#define PIN_ENABLE_3V3                      8
#define PIN_MODEM_SS                        6
#define PIN_MODEM_INT                       2

typedef uint32_t BaseType_t;
typedef uint16_t Msg_UID_t;
typedef uint8_t Node_UID_t;

#define NODE_UID_SIZE               sizeof(Node_UID_t)
#define MESG_UID_SIZE               sizeof(Msg_UID_t)


// +----------+------+------+-------------+-------------------------+----------+--------------+---------+-------------+----------------------+-----------------+
// |    0     |  2   |  3   |      4      |            6            |    8     |      9       |    9    |             |                      |                 |
// +----------+------+------+-------------+-------------------------+----------+--------------+---------+-------------+----------------------+-----------------+
// | MESG_UID | TYPE | HOPS | CUMMUL. LQI | NEXT_UID = PREVIOUS_UID | NODE_UID | PAYLOAD_SIZE | PAYLOAD | (EXTRA_UID) | (EXTRA_PAYLOAD_SIZE) | (EXTRA_PAYLOAD) |
// +----------+------+------+-------------+-------------------------+----------+--------------+---------+-------------+----------------------+-----------------+
//                                                                  | Payload starts here

#define HEADER_MESG_UID_OFFSET          0
#define HEADER_PAYLOAD_OFFSET           (HEADER_MESG_UID_OFFSET + MESG_UID_SIZE)
#define HEADER_SIZE                     HEADER_PAYLOAD_OFFSET

#define PAYLOAD_NODE_UID_OFFSET         0
#define PAYLOAD_DATA_OFFSET             (PAYLOAD_NODE_UID_OFFSET + NODE_UID_SIZE)

typedef void (*MsgReceivedCb)(uint8_t *, uint8_t);

class LoRaPTP{
    public:
        LoRaPTP();
        bool begin( void );
        void loop( void );
        bool sendMessage(uint8_t * payload, uint8_t len);
        void setMsgReceivedCb(MsgReceivedCb cb);
        void reconfigModem(void);

    private:
        bool handleAnyRxMessage(uint8_t * buf, uint8_t len);
        bool waitCADDone( void );
        bool waitRXAvailable(uint16_t timeout);
        void txMessage(uint8_t len);

        bool setFieldInBuffer(BaseType_t field, uint8_t * buf, uint8_t fieldOffset, size_t size);

        bool txPending;
        unsigned long txTime;
        uint8_t txLen;
        uint8_t txBuf[TX_BUFFER_SIZE];
        uint8_t rxBuf[RH_RF95_MAX_MESSAGE_LEN];

        Node_UID_t uid;
};

#endif	/* __LORA_PTP__ */
