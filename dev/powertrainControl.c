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
    //all four inverters have to be RTD before any torque is allowed
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

    //Undifferentiated driver request, identical for both rear motors
    baseCommand = (sbyte4)(throttlePercent * MAX_MOTOR_CURRENT_MA);

    if (me->powertrainMode == TorqueVectoring && sdiff != NULL)
    {
        //Software differential: derates the inside wheel based on steering angle.
        //s_diff_control returns commands in the same units it was handed (mA here).
        SDiff_Command sdiffCommand = s_diff_control(sdiff, (float4)steering_degrees(), (float4)baseCommand);
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
