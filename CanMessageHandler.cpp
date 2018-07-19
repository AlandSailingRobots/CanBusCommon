/****************************************************************************************
 *
 * File:
 *    CanMessageHandler.cpp
 *
 * Purpose:
 *    The purpose of this class is a unified use of CanMsg handling
 *    from both Arduino and RPI
 *
 * Developer Notes:
 *    There is only 7 bytes of data that can be encoded by using this class,
 *    because the last byte of the CanMsg is reserved for an error message.
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

std::bitset<64> CanMessageHandler::getMessageInBitset() {
    return m_message_bitset;
}

uint8_t CanMessageHandler::getErrorMessage() {
    uint8_t errorMessage;
    switch(m_message.id) {

        case MSG_ID_CURRENT_SENSOR_REQUEST:
            getData(&errorMessage, CURRENT_SENSOR_ERROR_START, 
                     CURRENT_SENSOR_ERROR_DATASIZE, CURRENT_SENSOR_ERROR_IN_BYTE);
            break;

        case MSG_ID_MARINE_SENSOR_DATA:
            getData(&errorMessage, SENSOR_ERROR_START, SENSOR_ERROR_DATASIZE, SENSOR_ERROR_IN_BYTE);
            break;

        default:
            errorMessage = m_message.data[INDEX_ERROR_CODE];
            break;

    }
    return errorMessage;
}

void CanMessageHandler::setErrorMessage(uint8_t errorMessage) {

    switch(m_message.id) {

        case MSG_ID_CURRENT_SENSOR_REQUEST:
            if(errorMessage >= 8) {
                #ifndef ON_ARDUINO_BOARD
                Logger::error("In CanMessageHandler::setErrorMessage(): error code value > current_sensor_max_error_value. Wrong error coded.");
                #endif
                // encode error 7('111') to make sure an error is still coded
                encodeMessage(7, CURRENT_SENSOR_ERROR_START, 
                               CURRENT_SENSOR_ERROR_DATASIZE, CURRENT_SENSOR_ERROR_IN_BYTE); 
            } else {
                encodeMessage(errorMessage, CURRENT_SENSOR_ERROR_START,
                               CURRENT_SENSOR_ERROR_DATASIZE, CURRENT_SENSOR_ERROR_IN_BYTE);
            }
            break;

        case MSG_ID_MARINE_SENSOR_DATA:
            encodeMessage(errorMessage, SENSOR_ERROR_START, SENSOR_ERROR_DATASIZE, SENSOR_ERROR_IN_BYTE);
            break;

        default:  // other part of the code still use this version
            if (m_message.data[INDEX_ERROR_CODE] == NO_ERRORS) {
                m_message.data[INDEX_ERROR_CODE] = errorMessage;
            }
            break;
                   
    }

}

bool CanMessageHandler::canMsgToBitset() {
    m_message_bitset = 0;

    m_message_bitset |= (static_cast<std::bitset<64>>(m_message.data[7]));  // Arduino crash when shifting a bitset by zero...
                                                              // So we do the first iteration before the loop in this case

    for(int i=1; i<8; i++){
        m_message_bitset |= (static_cast<std::bitset<64>>(m_message.data[7-i])) << i*8;
    }

    if(!(m_message_bitset.any())){ // In case of overflow and some other wrong operations, the returned bitset is zeros only
        // Check if we are compiling for arduino board, so we don't use the logger on it, arduino use AVR architecture. 
        #ifndef ON_ARDUINO_BOARD
        Logger::error("In CanMessageHandler::canMsgToBitset(): Data bits are unset, most likely a wrong operation");
        #endif 

        return false;
    }
    return true;
}

bool CanMessageHandler::bitsetToCanMsg() { // no false output at the moment
    for(int i=0; i<8; i++) {
        m_message.data[i] = 0; // reset here before copying in the bitset
        getData(&(m_message.data[i]), 7-i, 1, true);
    }

    return true;
}

/*bool generateHeader(int msgType) {
    // Add definition of canMsgType or just use the already defined ID defs
    switch (canMsgType) {
        case MSG_ID_CURRENT_SENSOR_REQUEST: // temporarily using this value because it's unused 
            generateCurrentSensorHeader();
            encodeMessage(T data, uint start, uint length, bool varInBytes = true)
            return true;
    }
}*/

bool CanMessageHandler::generateCurrentSensorHeader(uint8_t sensorID, uint8_t rolling_number) {
    // Current sensor header infos:   ID   | rol_num | error_value
    // Number of bits             :    3        2           3
    bool success = true;
    bool varInBytes = false;
    success &= encodeMessage(sensorID, 7*8 + 5, 3, varInBytes); // May define things like current_sensor_id_start_bits etc.
    success &= encodeMessage(rolling_number, 7*8 + 3, 2, varInBytes);
    return success;
}

