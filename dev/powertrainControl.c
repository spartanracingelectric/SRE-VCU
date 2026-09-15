/*****************************************************************************
 * powertrainControl.c
 * Formerly (AMKDrive.h - Drive Inverter (DI))
 * Author: Akash Karthik
 ******************************************************************************
 * Calculates torque to send to custom inverters
 ****************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_Driver.h"
#include "IO_CAN.h"

#include "powertrainControl.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensorCalculations.h"
#include "daqSensors.h"

extern Sensor Sensor_HVILTerminationSense;



_Powertrain* Powertrain_new(){
    _Powertrain* me = (_Powertrain*)malloc(sizeof(_Powertrain));
    ubyte1 motorIndex;
    ubyte4 maxCurrent_mA = 125000; //125A * 1000

        me->powertrainMode = MVP;

        for (motorIndex = 0; motorIndex < MOTOR_COUNT; motorIndex++)
        {
            me->motor[motorIndex].canId = MOTOR_VESC_ID_UNASSIGNED;
            me->motor[motorIndex].voltage_dV = 0;
            me->motor[motorIndex].current_dA = 0;
            me->motor[motorIndex].rpm = 0;
            me->motor[motorIndex].commandCurrent_mA = 0;
        }

        me->motor[MOTOR_RL].canId = 1;
        me->motor[MOTOR_RR].canId = 0;
    return me;
}

void Powertrain_ParseCanMessage(_Powertrain* me, IO_CAN_DATA_FRAME* canMessage){
    ubyte1 motorIndex;
    for (motorIndex = 0; motorIndex < MOTOR_COUNT; motorIndex++)
    {
        if (me->motor[motorIndex].canId == canMessage->id)
        {
            me->motor[motorIndex].voltage_dV = (canMessage->data[0] << 8) | canMessage->data[1];
            me->motor[motorIndex].current_dA = (canMessage->data[2] << 8) | canMessage->data[3];
            me->motor[motorIndex].rpm = (canMessage->data[4] << 24) | (canMessage->data[5] << 16) | (canMessage->data[6] << 8) | canMessage->data[7];
            break;
        }
    }
}


void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps){
    float4 throttlePercent = tps->travelPercent;
    if (throttlePercent < 0.0f)
        throttlePercent = 0.0f;
    else if (throttlePercent > 1.0f)
        throttlePercent = 1.0f;
    
    sbyte4 driverRequestedCurrent_mA = throttlePercent * maxCurrent_mA;

    me->motor[MOTOR_RL].commandCurrent_mA = driverRequestedCurrent_mA;
    me->motor[MOTOR_RR].commandCurrent_mA = driverRequestedCurrent_mA;

    return;

}
