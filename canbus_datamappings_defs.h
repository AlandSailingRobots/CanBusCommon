/****************************************************************************************
*
* File:
* 		canbus_datamappings_defs.h
*
* Purpose:
*		 The purpose of this definitions is a unified use reserved datasize per data packet
*		 and a common interval for each data between Arduino and RPI.
*
* Developer Notes:
*
***************************************************************************************/

#ifndef SAILINGROBOT_CANBUS_DATAMAPPINGS_DEFS_H
#define SAILINGROBOT_CANBUS_DATAMAPPINGS_DEFS_H


const int SENSOR_PH_DATASIZE = 1;
const int SENSOR_PH_INTERVAL_MIN = 0;
const int SENSOR_PH_INTERVAL_MAX = 14;

const int SENSOR_CONDUCTIVETY_DATASIZE = 4;
const int SENSOR_CONDUCTIVETY_INTERVAL_MIN = 5;
const long int SENSOR_CONDUCTIVETY_INTERVAL_MAX = 200000;

const int SENSOR_TEMPERATURE_DATASIZE = 2;
const int SENSOR_TEMPERATURE_INTERVAL_MIN = -5;
const int SENSOR_TEMPERATURE_INTERVAL_MAX = 40;





#endif //SAILINGROBOT_CANBUS_DATAMAPPINGS_DEFS_H
