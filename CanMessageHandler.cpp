/****************************************************************************************
*
* File:
* 		CanMessageHandler.cpp
*
* Purpose:
*		 The purpose of this class is a unified use of CanMsg handling
 *		 from both Arduino and RPI
 *
 *		 NOTE:
 *		 There is only 7 bytes of data that can be encoded by using this class,
 *		 because the last byte of the CanMsg is reserved for an error message.
*
* Developer Notes:
*
***************************************************************************************/

#include "CanMessageHandler.h"

CanMessageHandler::CanMessageHandler(CanMsg message) : m_message(message){

}


CanMessageHandler::CanMessageHandler(uint32_t messageId) {
    m_message.id = messageId;
    m_message.header.ide = 0;
    m_message.header.length = 8;
    for(auto& byteData : m_message.data) {
        byteData = 0;
    }

    m_message.data[INDEX_ERROR_CODE] = NO_ERRORS;
}

uint32_t CanMessageHandler::getMessageId() {
    return m_message.id;
}

CanMsg CanMessageHandler::getMessage() {
    return m_message;
}

uint8_t CanMessageHandler::getErrorMessage() {
    return m_message.data[INDEX_ERROR_CODE];
}

void CanMessageHandler::setErrorMessage(uint8_t errorMessage) {
    if(m_message.data[INDEX_ERROR_CODE] == NO_ERRORS) {
        m_message.data[INDEX_ERROR_CODE] = errorMessage;
    }
}

bool encodeCSMessage(int lengthInBytes, float data, uint8_t position) {
// Kind of encode Float Message now, where we choose 16 or 32 bits.
// position is 0 or 1 (from lower bit to higher bit -> 0 is right half), length is 16 or 32.
// If length is 32, position value is not used (but still rise error if not 0 or 1).
    Float16Compressor fltCompressor;
    int bitMaskLeft  = 0xffff0000;
    int bitMaskRight = 0x0000ffff;
    if (position>=2) {
        // Create error message?
        //setErrorMessage(SOMETHING);
    } else {
        if (lengthInBytes==16) {
            uint16_t compressedFloatData = fltCompressor.compress(data);
            if (position==0) {
                m_message.data += 
            }
        } else if (lengthInBytes==32) {
            
        }
    }


for (int i = 0; i < lengthInBytes; i++) {
  int dataIndex = currentDataWriteIndex + i;
  m_message.data[dataIndex] = (data >> 8 * i) & 0xff;
}
currentDataWriteIndex += lengthInBytes;
return true;
}


bool getCSData(float *dataToSet, int lengthInBytes) {
*dataToSet = 0;
unsigned long tmp_data_holder = 0;
if (currentDataReadIndex + lengthInBytes > MAX_DATA_INDEX + 1) {
//Serial.println("getData entering error condition");
  return false;
}
for (int i = 0; i < lengthInBytes; i++) {
  tmp_data_holder = (uint32_t)m_message.data[currentDataReadIndex + i] << (i * 8);
  *dataToSet += (T)(tmp_data_holder);
}
currentDataReadIndex += lengthInBytes;

return *dataToSet != static_cast<T>(DATA_NOT_VALID);
}
