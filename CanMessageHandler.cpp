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
    m_message.data[INDEX_ERROR_CODE] = errorMessage;
}


