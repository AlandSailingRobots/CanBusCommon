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
    //canMsgToBitset(); // to make sure bitset is updated
    return m_message_bitset;
}

uint8_t CanMessageHandler::getErrorMessage() {
    return m_message.data[INDEX_ERROR_CODE];
}

bool CanMessageHandler::setErrorMessage(uint8_t errorMessage) {
    if(m_message.id==MSG_ID_CURRENT_SENSOR_REQUEST) {
        // error code has 3 bits in current sensor data
        if(errorMessage >= 8) {
            #ifndef ON_ARDUINO_BOARD
            Logger::error("In CanMessageHandler::setErrorMessage(): error code value > current_sensor_max_error_value. Wrong error coded.");
            #endif
            encodeMessage(7, 7*8, 3, false); // encode error 7('111') to make sure an error is still coded
            return false;
        } else {
            encodeMessage(errorMessage, 7*8, 3, false);
            return true;
        }

    }
    else if(m_message.data[INDEX_ERROR_CODE] == NO_ERRORS) { // using if/elseif to keep the older parts of the code working 
        m_message.data[INDEX_ERROR_CODE] = errorMessage;     // most likely used by marine sensors only, have to check that.
        return true;
    }
    else {
        return false; // no error has been set
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
    //m_message.data = {0,0,0,0,0,0,0,0};
    //std::fill(m_message.data, m_message.data+8, 0);
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

