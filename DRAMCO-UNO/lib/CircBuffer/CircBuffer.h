#ifndef __CIRC_BUFFER_H__
#define	__CIRC_BUFFER_H__

#include <Arduino.h>

#define MAX_BUF_SIZE 16

typedef struct msgInfo{
  //  uint16_t nodeUid;
    uint16_t msgUid;
} MsgInfo_t;

typedef MsgInfo_t BufferType_t;

typedef enum circBufferStatuses{
	CB_SUCCESS,
	CB_EMPTY,
	CB_NOT_INITIALIZED,
	CB_NOT_NULL,
	CB_ARG_NULL,
    CB_NOT_IN_BUFFER,
	CB_ERROR
} CircBufferStatus_t;

class CircBuffer{
	public:
		CircBuffer(void);
		CircBufferStatus_t init(uint8_t length);
		void reset(void);
		bool isEmpty(void);
		uint8_t getFill();
		CircBufferStatus_t put(BufferType_t data);
		CircBufferStatus_t find(BufferType_t data);
        void print(void);
	
	private:
		BufferType_t * buffer;
		uint8_t head;
		uint8_t tail;
		uint8_t length;
		uint8_t fill;
};

#endif	/* __CIRC_BUFFER_H__ */

