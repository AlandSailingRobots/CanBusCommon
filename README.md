# Commons Library #
This library contains code and definitions that are used from both Arduino and Raspberry Pi
You will need to install the ArduinoSTL library. It's pretty straightforward using the library
manager from the Arduino IDE.

* This version needs the ArduinoSTL library. In ArduinoIDE go to "Sketch > Include Library > Manage Libraries..." and install ArduinoSTL from there.

## Important Info ##

* If you change any definitions you will have to recompile all code for both Arduino and RPI to get the updates up and running.

## Can message info ##
* To create a brand new Can message the following files are what should be changed

* Add the message id to the canbus_id_defs.h

* Add any new error messages to the canbus_error_defs.h

* Add new data interval values and data size constants to the canbus_datamappings_defs.h


## Can message basics ##

* Below example shows how to encode/decode data in a canmsg.

* NOTE: if you want to handle exact integer values the functions encodeMessage(LENGTH, DATA) and getData(*DATA, LENGTH) should be used instead. Further documentation exist in the CanMessageHandler.h file

```c++
const int MESSAGE_ID = 435;

const int DATA_INTERVAL_MIN = 0;
const int DATA_INTERVAL_MAX = 13;
const int DATASIZE_IN_BYTES = 2; // data will be encoded to this datasize

CanMessageHandler messageHandler(MESSAGE_ID);

float actualData = 4.56;

/*
This will map the data 4.56 to the interval 0 - 13 by using 2 bytes of the CanMsg
*/
boolean successful = messageHandler.encodeMappedMessage(DATASIZE_IN_BYTES, actualData, DATA_INTERVAL_MIN, DATA_INTERVAL_MAX);
CanMsg message = messageHandler.getMessage();


/*
Get data from message
*/

CanMessageHandler messageBackHandler(message);
float backData = 0;

boolean successful = messageBackHandler.getMappedData(&backData, DATASIZE_IN_BYTES, DATA_INTERVAL_MIN, DATA_INTERVAL_MAX);

// backData should now be 4.56

```

## Can message basics version 2 ##

* You add the position in the 64 bits of data, either in bytes or bits, the use is the same as it overrides the previously explained version

```c++
const int MESSAGE_ID = 435;


const int DATASIZE_IN_BYTES = true; // the position and length constants are interpreted as bytes
const int DATASIZE_IN_BYTES = false;// the position and length constants are interpreted as bits

// Let's say we want a data taking 2 bytes with its starting position on byte 4
const int SENSOR_DATA_IN_BYTES = true;
const int SENSOR_DATA_DATASIZE = 2;
const int SENSOR_DATA_START = 4;

// Let's say we want a data taking 7 bits with its starting position on byte 13
const int SENSOR_DATA_IN_BYTES = false;
const int SENSOR_DATA_DATASIZE = 7;
const int SENSOR_DATA_START = 13;

CanMessageHandler messageHandler(MESSAGE_ID);

float actualData = 4.56;
// You can compress the float into a half-precision representation.
// This way the float will be encoded in 2 bytes instead of 4.
#include <Float16Compressor.h>
Float16Compressor fltCompressor;  // Half precision float converter, IEEE754 standard
uint16_t compressedData = fltCompressor.compress(actualData);

/*
This will map the data 4.56 using 2 bytes (half-precision float) of the CanMsg
*/
messageHandler.encodeMessage(cmp_temperature, SENSOR_DATA_START, SENSOR_DATA_DATASIZE, SENSOR_DATA_IN_BYTE);



```


