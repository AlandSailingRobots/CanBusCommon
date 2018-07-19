/****************************************************************************************
 *
 * File:
 *    CanMessageHandler.h
 *
 * Purpose:
 *    The purpose of this class is a unified use of CanMsg handling
 *    from both Arduino and RPI
 *
 * NOTE:
 *    There is only 7 bytes of data that can be encoded by using this
 *    class, because the last byte of the CanMsg is reserved for an error message.
 *
 * Developer Notes:
 *    WARNING: ArduinoStl library used have no to_ullong function, we are limited to 4 bytes read (getData()) on Arduino boards.
 *             On the Raspberry PI side there is no problem for 8 bytes read.
 *    NEED to install ArduinoSTL, easy to do from ArduinoIDE with the library manager
 *
 ***************************************************************************************/

#ifndef SAILINGROBOT_CANMESSAGEHANDLER_H
#define SAILINGROBOT_CANMESSAGEHANDLER_H

#include <stdint.h>

#include "Float16Compressor.h"
#include "CanUtility.h"
#include "canbus_defs.h"

#if (defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_NANO))
 #define ON_ARDUINO_BOARD
#endif

#ifdef ON_ARDUINO_BOARD    // NEED to install ArduinoSTL library, easy to do from ArduinoIDE with the library manager
 #include <math.h>
 #include <bitset>
 #include <type_traits>
 typedef unsigned int uint; // Arduino have no native typedef from unsigned int to uint
#else
 #include <bits/stdc++.h>
 #include "../../../SystemServices/Logger.h"
#endif // ON_ARDUINO_BOARD

class CanMessageHandler {
   private:
    const int DATA_NOT_VALID = 0;
    const int MAPPING_INTERVAL_START = 1;

    const int MAX_DATA_INDEX = 6;
    const int INDEX_ERROR_CODE = 7;

    int currentDataWriteIndex = 0;
    int currentDataReadIndex = 0;

    CanMsg m_message;
    std::bitset<64> m_message_bitset;
    // NOTE: m_message and m_message_bitset can currently be filled separately

   public:
    /**
     * Class constructor
     *
     * Initializes a new clean CanMsg
     *
     * @param messageId the message id of CanMsg
     */
    explicit CanMessageHandler(uint32_t messageId);

    /**
     * Class constructor
     *
     * Initializes a Message handler for the CanMsg given to constructor
     *
     * @param message
     */
    explicit CanMessageHandler(CanMsg message);

    /**
     * Returns this messages ID
     * That ID SHOULD exist in canbus_id_defs.h
     *
     * @return message ID
     */
    uint32_t getMessageId();

    /**
     * Retrieves the constructed CanMsg from handler
     * @return the current CanMsg
     */
    CanMsg getMessage();

    /**
     * Retrieves the constructed CanMsg from handler
     * @return the current CanMsg
     */
    std::bitset<64> getMessageInBitset(); // rename to getBitsetMessage (currently filled separately from message)

    /**
     * Get an value between 0 - 255 used as an error message
     * @return the inserted error message from CanMsg
     */
    uint8_t getErrorMessage();

    /**
     * Inserts a value used as a error message.
     * Intended to be used with the error definitions in canbus_error_defs.h
     * @param errorMessage A value between 0 - 255
     */
    void setErrorMessage(uint8_t errorMessage);

    /**
     * Converts CanMsg.data into a bitset, hold by m_message_bitset
     */
    bool canMsgToBitset();

    /**
     * Converts bitset to CanMsg.data
     */
    bool bitsetToCanMsg();

    /**
     * Generate current sensor header: sensorID(3 bits) | rolling number(2 bits) | error(3 bits) 
     */
    bool generateCurrentSensorHeader(uint8_t sensorID, uint8_t rolling_number);

    /**
     * Function to retrieve data from the CanMsg.
     * Class contains a internal index counter so there is no need to keep track
     * of index positions
     *
     * @param lengthInBytes the number of bytes you want to retrieve
     * @param dataToSet a pointer to the data to set
     * @return false if data is not valid or exceeding the index bounds
     */
    template <class T> // need to keep it for some actuators code
    bool getData(T* dataToSet, int lengthInBytes) {
        *dataToSet = 0;
        unsigned long tmp_data_holder = 0;

        if (currentDataReadIndex + lengthInBytes > MAX_DATA_INDEX + 1) {
            // Serial.println("getData entering error condition");
            return false;
        }

        for (int i = 0; i < lengthInBytes; i++) {
            tmp_data_holder = (uint32_t)m_message.data[currentDataReadIndex + i] << (i * 8);
            *dataToSet += (T)(tmp_data_holder);
        }
        currentDataReadIndex += lengthInBytes;

        return *dataToSet != static_cast<T>(DATA_NOT_VALID);
    }

    // WARNING: this version get the data from the bitset only
    // WARNING: use ONLY with UNSIGNED types to avoid random behavior
    template <class T> 
    bool getData(T *dataToSet, uint start, uint length, bool varInBytes = true) {
        #ifndef ON_ARDUINO_BOARD
        if(!std::is_unsigned<T>::value) {
            Logger::warning("In CanMessageHandler::getData(): Casting to SIGNED type, can lead to wrong data!");   
        }
        #endif
        if(varInBytes) { length *= 8; start  *= 8; }

        std::bitset<64> data_container; // init to zero
        //std::bitset<64> mask((pow(2,length+start)-1)-(pow(2,start)-1));
        std::bitset<64> mask((pow(2,length)-1));

        if(start > 0) {  // Arduino fails to shift by zero a bitset...
            mask <<= start;
            data_container = (m_message_bitset & mask) >> start;
        } else {
            data_container = (m_message_bitset & mask);
        }
        
        #ifndef ON_ARDUINO_BOARD
        *dataToSet = static_cast<T>(data_container.to_ullong()); // NOTE: could add an option to return a bitset or not?
        #else
        // WARNING: ArduinoStl library used have no to_ullong function, we are limited to 4 bytes read on Arduino boards.
        // NOTE   : please find a cleaner solution. Current solution consists in using the bitset constructor from a
        //          string, so we convert the bitset<64> into string and then move its c_str() pointer by 32 so we have
        //          the right side half of the bitset<64> used for the bitset<32> constructor.
        //          The additional explicit declarations of templates are required by the arduino as the compiler have a
        //          hard time resolving them by itself.
        std::bitset<32> arduino_sized_bitset(static_cast<std::string>(data_container.to_string<char, std::string::traits_type, std::string::allocator_type>().c_str()+32));
        *dataToSet = static_cast<T>(arduino_sized_bitset.to_ulong());  
        #endif

        if (start + length > 64) { // mask will be zero in this case
            #ifndef ON_ARDUINO_BOARD
            Logger::error("In CanMessageHandler::getData(): Wrong reading parameters");
            #endif
            return false;
        }
        if(!(data_container.any())){ // In case of overflow and some other wrong operations, the returned bitset is zeros only,
                                     // but it can be a data in some cases.
            #ifndef ON_ARDUINO_BOARD
            // This log appears too many times when nothing wrong is happening, commented for now
            // Logger::warning("In CanMessageHandler::getData(): Data bits are unset");
            #endif
            return false;
        }

        return true;        
    }

    /**
     * Function to retrieve data from CanMsg and interpret
     * them to the value they had before they were inserted into CanMsg.
     *
     * NOTE:
     *       Due to casting back and forth floating numbers this method should not
     * be used if you want a precise integer number. In that case use the standard
     * getData method
     *
     * IMPORTANT NOTE:
     *       This method cannot handle data larger than 4 bytes due to the
     * uint32_t type below
     *
     * @param dataToSet a pointer to the data to set
     * @param lengthInBytes the number of bytes you want to retrieve
     * @param minValue The lower part of the interval you want to interpret data
     * to
     * @param maxValue The higher part of the interval you want to interpret data
     * to
     * @return false if data is not valid
     */
    template <class T> // AVOID USING FOR THE MOMENT, NO LONGER WORKS AFTER MODS ON getData()
    bool getMappedData(T* dataToSet, int lengthInBytes, long int minValue, long int maxValue) {

        uint32_t data;
        bool success = getData(&data, lengthInBytes); 

        if (success) {
            auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes) - 1;
            *dataToSet = static_cast<T>(CanUtility::mapInterval(
                data, MAPPING_INTERVAL_START, possibilitiesDataCanHold, minValue, maxValue));
            return true;
        } else {
            *dataToSet = static_cast<T>(DATA_NOT_VALID);
            return false;
        }
    }

    // New version, limited to 32 bits if called by an arduino
    template <class T> 
    bool getMappedData(T* dataToSet, uint start, uint length, bool varInBytes, long int minValue, long int maxValue) {
        #ifdef ON_ARDUINO_BOARD
        uint32_t data;
        #else
        uint64_t data;
        #endif

        bool success = getData(&data, start, length, varInBytes);

        if (success) {
            if (varInBytes) { length *= 8; start *= 8; }
            uint32_t maxValueFittingInGivenLength = (pow(2, length) - 1);
            *dataToSet = static_cast<T>(CanUtility::mapInterval(
                data, 0, maxValueFittingInGivenLength, minValue, maxValue));
            return true;
        } else {
            *dataToSet = static_cast<T>(DATA_NOT_VALID);
            return false;
        }
    }

    /**
     * Encodes a clean positive integer value into canMsg.
     * Note: data value MUST be within the range of the lengthInBytes parameter
     *
     * Class contains an data index counter which is incremented after every
     * insert so no need of keeping track of index
     *
     * @tparam T The data type used, must be a positive integer value.
     * @param lengthInBytes The number of bytes this data requires
     * @param data The data that needs to be encoded into the CanMsg
     * @return false if there is no more room in CanMsg
     */
    template <class T>
    bool encodeMessage(int lengthInBytes, T data) {

        if (currentDataWriteIndex + lengthInBytes > MAX_DATA_INDEX + 1) {
            setErrorMessage(ERROR_CANMSG_INDEX_OUT_OF_INTERVAL);
            return false;
        }

        for (int i = 0; i < lengthInBytes; i++) {
            int dataIndex = currentDataWriteIndex + i;
            m_message.data[dataIndex] = (data >> 8 * i) & 0xff;
        }
        currentDataWriteIndex += lengthInBytes;
        return true;
    }

    // @data MUST be an unsigned int to avoid potential random behavior
    // NOTE: ONLY ENCODING THE BITSET at the moment
    template <class T>
    bool encodeMessage(T data, uint start, uint length, bool varInBytes = true) {
            /* IMPLEMENT THIS LATER
        int ERROR_CANMSG_ENCODING_OUT_OF_BOUND = 2;
        int ERROR_CANMSG_MASK_HAS_NO_BIT_SET = 3;
        int ERROR_CANMSG_OVERWRITING = 4; // more a warning than an error  */
        #ifndef ON_ARDUINO_BOARD
        if(std::is_unsigned<T>::value) {
            Logger::warning("In CanMessageHandler::encodeMessage(): Casting to SIGNED type, can lead to wrong data!");
        }
        #endif
        // add an if check: if b_data >> max value --> overflow
        if (varInBytes) { length *= 8; start *= 8; } // for simpler access using bytes
        std::bitset<64> b_data(data); // if data is a signed int, it'll get through a ulong or ullong cast

        // NOTE: Masks usage disabled for encoding as it leads to data corruption on the arduino side
        //std::bitset<64> mask(pow(2,length)-1);

        if(start > 0){ // again Arduino can't shift this bitset by zero...
            //mask <<= start;

            // Shift and use mask(just later). IMPORTANT: if length is not high enough, or start is too high, data corruption will occur here
            b_data <<= start;
        }
/*
        if ((m_message_bitset&mask).any()) { // simple check, does not take into account that a zero could be data
            #ifndef ON_ARDUINO_BOARD
            Logger::warning("Warning, overwriting data in the container");
            #endif
            //setErrorMessage(ERROR_CANMSG_OVERWRITING);
        }
*/        
        //b_data &= mask;

        // Set error message
        if (start > 64) {
            #ifndef ON_ARDUINO_BOARD
            Logger::error("In CanMessageHandler::encodeMessage(): start > 64 ---> overflow!");
            #endif
            //setErrorMessage(ERROR_CANMSG_ENCODING_OUT_OF_BOUND);
            return false;
        }
/*        if (!mask.any()) { // Assuming we never want an unset mask, it is a way to catch some unmatching length and start value
                           // because mask is still 0 when overflowing at constructor step
            #ifndef ON_ARDUINO_BOARD
            Logger::error("In CanMessageHandler::encodeMessage(): Mask has no bits set, check LENGTH and START params.\n");
            #endif
            //setErrorMessage(ERROR_CANMSG_MASK_HAS_NO_BIT_SET);
            return false;
        }
*/
        // Merging to container
        m_message_bitset |= b_data;   
        return true;
    }

    /**
     * Encodes a value into canMsg, mapped onto the range given and the bytes
     * available. Note: A bigger range of data this leads to less precision A
     * smaller amount of length leads to less precision
     *
     *       Due to casting back and forth floating numbers this method should not
     * be used if you want a precise integer number. In that case use the standard
     * encode method
     *
     * Class contains an data index counter which is incremented after every
     * insert so no need of keeping track of index
     *
     * @tparam T The data type inserted
     * @param lengthInBytes The number of bytes this data requires
     * @param data The data that needs to be encoded into the CanMsg
     * @param minValue The lower part of the interval you want to encode data to
     * @param maxValue The higher part of the interval you want to encode data to
     * @return
     */
    template <class T>
    bool encodeMappedMessage(int lengthInBytes, T data, long int minValue, long int maxValue) {
        if (data > maxValue || data < minValue) {
            setErrorMessage(ERROR_CANMSG_DATA_OUT_OF_INTERVAL);
            return false;
        }

        auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes) - 1;
        // uint_64 cast before
        auto mappedData = static_cast<uint32_t>(CanUtility::mapInterval(
            data, minValue, maxValue, MAPPING_INTERVAL_START, possibilitiesDataCanHold));

        return encodeMessage(lengthInBytes, mappedData);
    }

    // Version using the new encodeMessage
    // WARNING: we're limited to 4 bytes <-- casting into uint32_t (arduino boards used don't handle uint64_t)
    template <class T>
    bool encodeMappedMessage(T data, uint start, uint length, bool varInBytes, long int minValue, long int maxValue) {
        if (data > maxValue || data < minValue) {
            //setErrorMessage(ERROR_CANMSG_DATA_OUT_OF_INTERVAL);
            return false;
        }

        //auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes) - 1;
        if(varInBytes) { start *= 8; length *= 8; varInBytes =false; }; 
        // NOTE: Set varInBytes to false on this step or the call to encodeMessage will re-multiply by 8 start and length

        uint32_t maxValueFittingInGivenLength = (pow(2, length) - 1);
        uint32_t mappedData = static_cast<uint32_t>(CanUtility::mapInterval(
            data, minValue, maxValue, 0, maxValueFittingInGivenLength));

        return encodeMessage(mappedData, start, length, varInBytes);
    }

   
     bool generateHeader(int msgType) ;
};

#endif  // SAILINGROBOT_CANMESSAGEHANDLER_H
