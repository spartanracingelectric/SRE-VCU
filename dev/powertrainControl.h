/*****************************************************************************
 * powertrainControl.h 
 * Formerly (AMKDrive.h - Drive Inverter (DI))
 * Author: Akash Karthik
 ******************************************************************************
 * Calculates torque to send to custom inverters
 ****************************************************************************/

#ifndef _POWERTRAINCONTROL_H
#define _POWERTRAINCONTROL_H

/***** Standard includes *****/
#include <stdlib.h> //Needed for malloc
#include "IO_CAN.h"

#include "IO_Driver.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "daqSensors.h"
#include "bms.h"


typedef enum _PowertrainMode {
    DISABLED = 0,
    FWD = 1,
    RWD = 2,
    AWD = 3,
    TorqueVectoring = 4,
    //Novelty Modes
    Drift = 5,
    MVP = 6
} PowertrainMode;

typedef struct _Powertrain {
    PowertrainMode powertrainMode;

    // Torque commands to each motor in mA (signed)
    sbyte4 motor_fl;
    sbyte4 motor_fr;
    sbyte4 motor_rl;
    sbyte4 motor_rr;

} _Powertrain;

_Powertrain* Powertrain_new();

void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps);
#endif
