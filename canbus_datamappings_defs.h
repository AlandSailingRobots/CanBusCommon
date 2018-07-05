/****************************************************************************************
 *
 * File:
 *    canbus_datamappings_defs.h
 *
 * Purpose:
 *    The purpose of this definitions is a unified use reserved
 *    datasize per data packet and a common interval for each data between Arduino
 *    and RPI.
 *
 * Developer Notes:
 *    Definitions like ******_IN_BYTE means the start and length value of the considered
 *    ****** object are in bytes if set to true, and in bits if set to false.
 *
 ***************************************************************************************/

#ifndef SAILINGROBOT_CANBUS_DATAMAPPINGS_DEFS_H
#define SAILINGROBOT_CANBUS_DATAMAPPINGS_DEFS_H

// Used by marine sensor data message
const int SENSOR_PH_START = 0;
const int SENSOR_PH_DATASIZE = 1;
const int SENSOR_PH_IN_BYTE = true;
const int SENSOR_PH_INTERVAL_MIN = 0;
const int SENSOR_PH_INTERVAL_MAX = 14;

    // we can encode directly the float instead of using mappedData as we
    // are using 4 bytes, and arduino float are 4 bytes
const int SENSOR_CONDUCTIVETY_START = 1;
const int SENSOR_CONDUCTIVETY_DATASIZE = 4;
const int SENSOR_CONDUCTIVETY_IN_BYTE = true;
const int SENSOR_CONDUCTIVETY_INTERVAL_MIN = -50;
const long int SENSOR_CONDUCTIVETY_INTERVAL_MAX = 200000;

    // we can either use the mappedData functions here, are go with the
    // float16 compressor as we are using 2 bytes
const int SENSOR_TEMPERATURE_START = 5;
const int SENSOR_TEMPERATURE_DATASIZE = 2;
const int SENSOR_TEMPERATURE_IN_BYTE = true;
const int SENSOR_TEMPERATURE_INTERVAL_MIN = -5; 
const int SENSOR_TEMPERATURE_INTERVAL_MAX = 40;
//-----------------------------------------------------------

// Used by marine sensor request message
const int REQUEST_CONTINOUS_READINGS_DATASIZE = 1;
const int REQUEST_READING_TIME_DATASIZE = 4;
//-----------------------------------------------------------

// Used by AU Control and AU Feedback messages
/**
 * Rudder should go from -30 to +30 degrees
 * which gives an effective range of 60.
 */
const int RUDDER_ANGLE_DATASIZE = 2;
const int MAX_RUDDER_ANGLE = 30;
const int MIN_RUDDER_ANGLE = -MAX_RUDDER_ANGLE;

/**
 * Windsail should go from -13 to 13 degrees
 * range is 26
 */
const int WINGSAIL_ANGLE_DATASIZE = 1;
const int MAX_WINGSAIL_ANGLE = 13;
const int MIN_WINGSAIL_ANGLE = -MAX_WINGSAIL_ANGLE;

const int WINDVANE_SELFSTEERING_DATASIZE = 2;
const int WINDVANE_SELFSTEERING_ANGLE_MAX = 360;
const int WINDVANE_SELFSTEERING_ANGLE_MIN = 0;

const int WINDVANE_ACTUATOR_POSITION_DATASIZE = 1;

const int WINDVANE_SELFSTEERING_ON_DATASIZE = 1;

//-----------------------------------------------------------

// Used by Radio Controller Status message
const int RADIOCONTROLLER_ON_DATASIZE = 1;
//-----------------------------------------------------------

// Used by Current Sensor message
const int CURRENT_SENSOR_CURRENT_DATASIZE = 2; // in bytes
const int CURRENT_SENSOR_CURRENT_START    = 2; // in bytes
const int CURRENT_SENSOR_CURRENT_IN_BYTE  = true;

const int CURRENT_SENSOR_VOLTAGE_DATASIZE = 2; // in bytes
const int CURRENT_SENSOR_VOLTAGE_START    = 0; // in bytes
const int CURRENT_SENSOR_VOLTAGE_IN_BYTE  = true;

const int CURRENT_SENSOR_ID_DATASIZE      = 3;       // in bits
const int CURRENT_SENSOR_ID_START         = 7*8 + 5; // in bits
const int CURRENT_SENSOR_ID_IN_BYTE       = false;

const int CURRENT_SENSOR_ROL_NUM_DATASIZE = 2;       // in bits
const int CURRENT_SENSOR_ROL_NUM_START    = 7*8 + 3; // in bits
const int CURRENT_SENSOR_ROL_NUM_IN_BYTE  = false;


#endif  // SAILINGROBOT_CANBUS_DATAMAPPINGS_DEFS_H
