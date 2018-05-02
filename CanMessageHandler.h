/****************************************************************************************
*
* File:
* 		CanMessageHandler.h
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

#ifndef SAILINGROBOT_CANMESSAGEHANDLER_H
#define SAILINGROBOT_CANMESSAGEHANDLER_H

#include <stdint.h>

#include "CanUtility.h"
#include "canbus_defs.h"

class CanMessageHandler {
private:
    const int DATA_NOT_VALID = 0;
    const int MAPPING_INTERVAL_START = 1;

    const int MAX_DATA_INDEX = 6;
    const int INDEX_ERROR_CODE = 7;

    int currentDataWriteIndex = 0;
    int currentDataReadIndex = 0;

    CanMsg m_message;
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
     * Function to retrieve data from the CanMsg.
     * Class contains a internal index counter so there is no need to keep track of index positions
     *
     * @param lengthInBytes the number of bytes you want to retrieve
     * @param dataToSet a pointer to the data to set
     * @return false if data is not valid or exceeding the index bounds
     */
    template<class T>
    bool getData(T* dataToSet, int lengthInBytes) {
        *dataToSet = 0;
        if(currentDataReadIndex+lengthInBytes > MAX_DATA_INDEX) {
            return false;
        }

        for (int i=0;i<lengthInBytes;i++) {
            *dataToSet += static_cast<T>(m_message.data[currentDataReadIndex+i] << i*8);
        }
        currentDataReadIndex += lengthInBytes;

        return *dataToSet != DATA_NOT_VALID;
    }

    /**
     * Function to retrieve data from CanMsg and interpret
     * them to the value they had before they were inserted into CanMsg.
     *
     *
     * @param dataToSet a pointer to the data to set
     * @param lengthInBytes the number of bytes you want to retrieve
     * @param minValue The lower part of the interval you want to interpret data to
     * @param maxValue The higher part of the interval you want to interpret data to
     * @return false if data is not valid
     */
    template<class T>
    bool getMappedData(T* dataToSet, int lengthInBytes, long int minValue, long int maxValue) {
        uint64_t data;
        bool success = getData(&data, lengthInBytes);

        if(success) {
            auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes)-1;
            *dataToSet = static_cast<T>(CanUtility::mapInterval(data, MAPPING_INTERVAL_START, possibilitiesDataCanHold, minValue, maxValue));
            return true;
        }
        else {
            *dataToSet = static_cast<T>(DATA_NOT_VALID);
            return false;
        }
    }






    /**
     * Encodes a clean positive integer value into canMsg.
     * Note: data value MUST be within the range of the lengthInBytes parameter
     *
     * Class contains an data index counter which is incremented after every insert
     * so no need of keeping track of index
     *
     * @tparam T The data type used, must be a positive integer value.
     * @param lengthInBytes The number of bytes this data requires
     * @param data The data that needs to be encoded into the CanMsg
     * @return false if there is no more room in CanMsg
     */
    template<class T>
    bool encodeMessage(int lengthInBytes, T data) {

        if(currentDataWriteIndex+lengthInBytes > MAX_DATA_INDEX) {
            setErrorMessage(ERROR_CANMSG_INDEX_OUT_OF_INTERVAL);
            return false;
        }

        for(int i=0;i<lengthInBytes;i++) {
            int dataIndex = currentDataWriteIndex+i;
            m_message.data[dataIndex] = (data >> 8*i) &0xff;
        }
        currentDataWriteIndex += lengthInBytes;
        return true;
    }

    /**
     * Encodes a value into canMsg, mapped onto the range given and the bytes available.
     * Note: A bigger range of data this leads to less precision
     *       A smaller amount of length leads to less precision
     *
     * Class contains an data index counter which is incremented after every insert
     * so no need of keeping track of index
     *
     * @tparam T The data type inserted
     * @param lengthInBytes The number of bytes this data requires
     * @param data The data that needs to be encoded into the CanMsg
     * @param minValue The lower part of the interval you want to encode data to
     * @param maxValue The higher part of the interval you want to encode data to
     * @return
     */
    template<class T>
    bool encodeMappedMessage(int lengthInBytes, T data, long int minValue, long int maxValue) {

        if(data > maxValue || data < minValue) {
            setErrorMessage(ERROR_CANMSG_DATA_OUT_OF_INTERVAL);
            return false;
        }

        auto possibilitiesDataCanHold = CanUtility::calcSizeOfBytes(lengthInBytes)-1;
        auto mappedData = static_cast<uint64_t>(
                CanUtility::mapInterval(data, minValue, maxValue, MAPPING_INTERVAL_START, possibilitiesDataCanHold));

        return encodeMessage(lengthInBytes, mappedData);
    }
};


#endif //SAILINGROBOT_CANMESSAGEHANDLER_H
