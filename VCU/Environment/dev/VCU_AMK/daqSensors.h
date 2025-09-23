/*****************************************************************************
 * daqSensors.h - DAQ object and parsing
 * Initial Author: Shinika B
 ******************************************************************************
 * Items taken coming from the DAQs CAN messages
 ****************************************************************************/

#ifndef _DAQSENSORS_H
#define _DAQSENSORS_H

#include <stdlib.h> 
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "sensors.h"
#include "initializations.h"

typedef struct DAQSensors {
    
    ubyte4 AccelX;
    ubyte4 AccelY;
    ubyte4 AccelZ;

    ubyte4 GyroX;
    ubyte4 GyroY;
    ubyte4 GyroZ;

    ubyte4 GpsSpeed;

    ubyte4 DetectionPCB;

} _DAQSensors;

_DAQSensors* DAQ_Sensor_new();

void DAQ_parseCanMessage(_DAQSensors* daq1, IO_CAN_DATA_FRAME* daqCANMessage);

#endif