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

#define MOTOR_COUNT 4
#define MOTOR_CAN_ID_UNASSIGNED 0xFFU

typedef enum _MotorIndex {
    MOTOR_FL = 0,
    MOTOR_FR = 1,
    MOTOR_RL = 2,
    MOTOR_RR = 3
} MotorIndex;

typedef struct _Motor {
    ubyte1 canId;
    ubyte2 voltage_dV;
    sbyte2 current_dA;
    sbyte4 rpm;
    sbyte4 commandCurrent_mA;
} Motor;


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

    Motor motor[MOTOR_COUNT];

} _Powertrain;

_Powertrain* Powertrain_new();

void Powertrain_ParseCanMessage(_Powertrain* me, IO_CAN_DATA_FRAME* canMessage);
void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps);
#endif
