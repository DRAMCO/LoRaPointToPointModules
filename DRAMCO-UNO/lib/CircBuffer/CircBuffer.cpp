#include "CircBuffer.h"

CircBuffer::CircBuffer(void){}

/* Initialize a circular buffer
 */
CircBufferStatus_t CircBuffer::init(uint8_t length){
	// check max length
	if(length+1 > MAX_BUF_SIZE){
		return CB_ERROR;
	}
	
	// check if buffer has not yet been initalized
	if(this->buffer != NULL){
		return CB_NOT_NULL;
	}

	// allocate memory for the buffer
	this->buffer = (BufferType_t*)malloc(sizeof(BufferType_t)*(length));
	if(this->buffer == NULL){
		return CB_NOT_INITIALIZED;
	}
	
	// initialize attributes
	this->length = length;
    this->reset();

	return CB_SUCCESS;
}

/* Reset the buffer
 */ 
void CircBuffer::reset(void){
	// re-init attributes (memory is not cleared)
	this->head = 0;
	this->tail = 0;
	this->fill = 0;
    memset(this->buffer, 0, sizeof(BufferType_t)*this->length);
}

/* Returns true when the buffer is empty, and false otherwise.
 */
bool CircBuffer::isEmpty(void){
	// We define empty as head == tail
	return (this->head == this->tail);
}

/* Returns the number of values stored in the buffer.
 */
uint8_t CircBuffer::getFill(void){
	return this->fill;
}

/* Put a value in the buffer
 */
CircBufferStatus_t CircBuffer::put(BufferType_t data){
	// check if buffer is initalized
	if(this->buffer != NULL){
		// store value at the head of the buffer
		memcpy(&(this->buffer[this->head]), &data, sizeof(BufferType_t));
		// move head
		this->head = (this->head + 1) % this->length;
        if(this->fill < this->length){
            this->fill++;
        }

		// check if last byte has been overwritten (pushed out)
		if(this->head == this->tail){
			// move tail
			this->tail = (this->tail + 1) % this->length;
		}

		return CB_SUCCESS;
	}

	return CB_NOT_INITIALIZED;
}

/* Find a value in the buffer
 */
CircBufferStatus_t CircBuffer::find(BufferType_t data){
	// check if buffer is initalized
	if(this->buffer == NULL){
		return CB_NOT_INITIALIZED;
	}
	// check if buffer is empty
	if(!this->isEmpty()){
        for(uint8_t i=0; i<this->fill; i++){
            uint8_t entryIndex = (this->head + i) % this->length;
            if(memcmp(&(this->buffer[entryIndex]), &data, sizeof(BufferType_t)) == 0){
                return CB_SUCCESS;
            }
        }

		return CB_NOT_IN_BUFFER;
	}

	return CB_EMPTY;
}

void CircBuffer::print(void){
    Serial.print("Head: ");
    Serial.println(this->head);
    Serial.print("Tail: ");
    Serial.println(this->tail);
    Serial.print("Fill: ");
    Serial.println(this->fill);

    Serial.println("Buf:");
    for(uint8_t i=0; i<this->length; i++){
        Serial.println(this->buffer[i].msgUid, HEX);
    }
}