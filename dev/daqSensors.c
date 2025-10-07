/*****************************************************************************
 * daqSensors.h - DAQ object and parsing
 * Initial Author: Shinika B
 ******************************************************************************
 * Items taken coming from the DAQs CAN messages
 ****************************************************************************/

#include <stdlib.h> 
#include "IO_CAN.h"
#include "IO_Driver.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "daqSensors.h"

_DAQSensors* DAQ_Sensor_new(){

    _DAQSensors* me = (_DAQSensors*)malloc(sizeof(_DAQSensors));

    me->AccelX = 0;
    me->AccelY = 0;
    me->AccelZ = 0;

    me->GyroX = 0;
    me->GyroY = 0;
    me->GyroZ = 0;

    me->GpsSpeed = 0;

    return me;

}

void DAQ_parseCanMessage(_DAQSensors* me, IO_CAN_DATA_FRAME* daqCANMessage){

    //Issue with it not subtracting the value correctly due to the DBC (offset required) -> -320 and -250 dont work
    //Compare it with 320 and 250

    //See if the scale is needed

    if(daqCANMessage->id == 0x400) {

        me->AccelX = (((ubyte4)daqCANMessage->data[1] << 8 | daqCANMessage->data[0]) - 320.0) * 0.01; //m/s2
        me->AccelY = (((ubyte4)daqCANMessage->data[3] << 8 | daqCANMessage->data[2]) - 320.0) * 0.01; //m/s2
        me->AccelZ = (((ubyte4)daqCANMessage->data[5] << 8 | daqCANMessage->data[4]) - 320.0) * 0.01; //m/s2
        
    } else if(daqCANMessage->id == 0x402) {
        
        me->GyroX = (((ubyte4)daqCANMessage->data[1] << 8 | daqCANMessage->data[0]) - 250.0) * 0.0078125; //deg/s
        me->GyroY = (((ubyte4)daqCANMessage->data[3] << 8 | daqCANMessage->data[2]) - 250.0) * 0.0078125; //deg/s
        me->GyroZ = (((ubyte4)daqCANMessage->data[5] << 8 | daqCANMessage->data[4]) - 250.0) * 0.0078125; //deg/s

    } else if (daqCANMessage->id == 0x401) {
        me->GpsSpeed = (((ubyte4)daqCANMessage->data[0] << 8 | daqCANMessage->data[1])) * 0.001; //kph 

    } else if (daqCANMessage->id == 0x403) {
        me->DetectionPCB = daqCANMessage->data[0]; // On/Off
    }
}