//"Include guard" - prevents this file from being #included more than once
#ifndef POWERLIMITING_H
#define POWERLIMITING_H

#include "IO_Driver.h"
#include "pid.h"
#include "powertrainControl.h"
typedef struct {
    PID_Controller* pid;
    float targetPower;
} PowerLimiting;

PowerLimiting* PowerLimiting_new(float targetPower);
void PowerLimiting_limitPower(PowerLimiting* pl, _DriveInverter* mcu, TorqueEncoder* tps);

#endif