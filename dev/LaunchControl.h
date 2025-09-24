
#ifndef _LAUNCHCONTROL_H
#define _LAUNCHCONTROL_H

#include "IO_Driver.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "PID.h"
#include "drs.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

typedef struct _LaunchControl {
    PID_Controller* pidTorque;
    float4 targetSlip;
    float4 estimatedSlip;
    sbyte2 torqueRequest;

    ubyte4 safteyTimer;
    ubyte1 lcReady;
    ubyte1 lcActive;
    ubyte1 flags;

    PID_Controller* pidSpeed;
    sbyte2 speedRequest;

    ubyte1 constantSpeedTestOverride; // flag for speed mode override
    sbyte2 overrideTestSpeedCommand;

    ubyte1 buttonDebug;
} LaunchControl;

LaunchControl *LaunchControl_new();
void LaunchControl_estimateSlipRatio(LaunchControl *lc, MotorController *mcm, WheelSpeeds *wss);
void LaunchControl_calculateTorqueCommand(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs);
void LaunchControl_checkState(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs);
ubyte1 LaunchControl_getStatus(LaunchControl *lc);
sbyte2 LaunchControl_getTorqueCommand(LaunchControl *lc);
void LaunchControl_initialTorqueCurve(LaunchControl* me, MotorController* mcm);
void LaunchControl_initialRPMCurve(LaunchControl* me, MotorController* mcm);
ubyte1 LaunchControl_getButtonDebug(LaunchControl *lc);
void LaunchControl_checkRotary(LaunchControl *me);
#endif //_LAUNCHCONTROL_H
