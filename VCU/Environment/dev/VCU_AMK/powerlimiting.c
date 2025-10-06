#include <stdlib.h>
#include <math.h>
#include "powerlimiting.h"

PowerLimiting* PowerLimiting_new(float targetPower){
    PowerLimiting* powerlimiting = (PowerLimiting*)malloc(sizeof(PowerLimiting));

    powerlimiting->targetPower = targetPower;
    powerlimiting->pid = PID_new(1, 0, 0, 0, 100, 231, 0, 0, 0);

    return powerlimiting;
}

void PowerLimiting_limitPower(PowerLimiting* pl, _DriveInverter* mcu, TorqueEncoder* tps){
    
    pl->pid->controllerMaxima = (tps->travelPercent * 231);
    float processVariable = mcu->AMK_TorqueCurrent + mcu->AMK_MagnetizingCurrent;
    float targetValue = pl->targetPower /*/ mcu->voltage*/;
    float outputVariable = mcu->AMK_TorqueSetpoint;
    float torqueRequest = PID_computeOutput(pl->pid, targetValue, processVariable, outputVariable);
    //Sets 
    // mcu->AMK_TorqueSetpoint = (sbyte2)torqueRequest;
}
