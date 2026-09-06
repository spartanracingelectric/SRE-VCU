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
        me->powertrainMode = MVP;
        me->motor_fl = 0; // torque
        me->motor_fr = 0;
        me->motor_rl = 0;
        me->motor_rr = 0;
    return me;
}


void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps){
    //all four inverters have to be RTD before any torque is allowed
    float4 throttlePercent = tps->travelPercent;

    if (throttlePercent < 0.0f)
    {
        throttlePercent = 0.0f;
    }
    else if (throttlePercent > 1.0f)
    {
        throttlePercent = 1.0f;
    }

    me->motor_rl = (sbyte4)(throttlePercent*125 * 1000.0f);
    me->motor_rr = (sbyte4)(throttlePercent*125 * 1000.0f);

    return;

}
