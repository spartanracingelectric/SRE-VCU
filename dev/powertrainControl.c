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


_Powertrain* Powertrain_new(){
    _Powertrain* me = (_Powertrain*)malloc(sizeof(_Powertrain));
        me->powertrainMode = TorqueVectoring;
        me->motor_fl = 0; // torque
        me->motor_fr = 0;
        me->motor_rl = 0;
        me->motor_rr = 0;
    return me;
}


void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps, SDiff *sdiff){

    float4 throttlePercent = tps->travelPercent;
    sbyte4 baseCommand;

    if (throttlePercent < 0.0f)
    {
        throttlePercent = 0.0f;
    }
    else if (throttlePercent > 1.0f)
    {
        throttlePercent = 1.0f;
    }

    if(throttlePercent < 0.01f)
    {
        me->motor_fl = 0;
        me->motor_fr = 0;
        me->motor_rl = 0;
        me->motor_rr = 0;
        return;
    }

    else if(throttlePercent >= 0.01f) {
        baseCommand = (sbyte4)((throttlePercent - 0.01f) * MAX_MOTOR_CURRENT_MA + 7000);
    }

    if (me->powertrainMode == TorqueVectoring && sdiff != NULL)
    {
        sbyte4 sas_deg;
        if (steering_degrees(&sas_deg) == FALSE)
        {
            sas_deg = 0;
        }
        SDiff_Command sdiffCommand = s_diff_control(sdiff, (float4)sas_deg, (float4)baseCommand);
        me->motor_rl = sdiffCommand.left;
        me->motor_rr = sdiffCommand.right;
    }
    else
    {
        me->motor_rl = baseCommand;
        me->motor_rr = baseCommand;
    }

    return;

}
