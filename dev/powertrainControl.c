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
#include "sdiff.h"


extern Sensor Sensor_HVILTerminationSense;

// helper fucntions so we can read data that takes up multiple bytes in the data array (BE means big endian for all of yall)
static ubyte2 Powertrain_readU16BE(const ubyte1* data)
{
    return ((ubyte2)data[0] << 8) | (ubyte2)data[1];
}

static sbyte4 Powertrain_readS32BE(const ubyte1* data)
{
    ubyte4 value = ((ubyte4)data[0] << 24)
                 | ((ubyte4)data[1] << 16)
                 | ((ubyte4)data[2] << 8)
                 | (ubyte4)data[3];

    return (sbyte4)value;
}


_Powertrain* Powertrain_new(){
    _Powertrain* me = (_Powertrain*)malloc(sizeof(_Powertrain));
    ubyte1 motorIndex;

        me->powertrainMode = TorqueVectoring;

        for (motorIndex = 0; motorIndex < MOTOR_COUNT; motorIndex++)
        {
            me->motor[motorIndex].canId = MOTOR_CAN_ID_UNASSIGNED;
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
    ubyte1 vescId = (ubyte1)(canMessage->id & 0xFF);
    ubyte1 packetId = (ubyte1)((canMessage->id >> 8) & 0xFF);

    for (motorIndex = 0; motorIndex < MOTOR_COUNT; motorIndex++)
    {
        if ((me->motor[motorIndex].canId != MOTOR_CAN_ID_UNASSIGNED)
         && (me->motor[motorIndex].canId == vescId))
        {
            switch (packetId)
            {
                case 9U:
                    if (canMessage->length >= 4)
                    {
                        // dividing by 5 cause the vesc sends eRPM and with 5 pole pairs we get rpm by dividing by 5
                        me->motor[motorIndex].rpm = Powertrain_readS32BE(&canMessage->data[0]) / 5; 
                    }
                    break;

                case 16U:
                    if (canMessage->length >= 6)
                    {
                        me->motor[motorIndex].current_dA = (sbyte2)Powertrain_readU16BE(&canMessage->data[4]);
                    }
                    break;

                case 27U:
                    if (canMessage->length >= 6)
                    {
                        me->motor[motorIndex].voltage_dV = Powertrain_readU16BE(&canMessage->data[4]);
                    }
                    break;

                default:
                    break;
            }

            break;
        }
    }
}


void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps, SDiff *sdiff){
    float4 throttlePercent = tps->travelPercent;
    sbyte4 baseCommand;

    if (throttlePercent < 0.0f)
        throttlePercent = 0.0f;
    else if (throttlePercent > 1.0f)
        throttlePercent = 1.0f;

    baseCommand = (sbyte4)(throttlePercent * MAX_MOTOR_CURRENT_MA);

    if (me->powertrainMode == TorqueVectoring && sdiff != NULL)
    {
        sbyte4 sas_deg;
        SDiff_Command sdiffCommand;

        if (steering_degrees(&sas_deg) == FALSE)
        {
            sas_deg = 0;
        }

        sdiffCommand = s_diff_control(sdiff, (float4)sas_deg, (float4)baseCommand);
        me->motor[MOTOR_RL].commandCurrent_mA = sdiffCommand.left;
        me->motor[MOTOR_RR].commandCurrent_mA = sdiffCommand.right;
    }
    else
    {
        me->motor[MOTOR_RL].commandCurrent_mA = baseCommand;
        me->motor[MOTOR_RR].commandCurrent_mA = baseCommand;
    }

    return;

}
