#include <stdlib.h>
#include <stdlib.h>
#include <math.h>
#include "powerlimiting.h"
#include "motorController.h"

PowerLimiting* PowerLimiting_new(float targetPower){
    PowerLimiting* powerlimiting = (PowerLimiting*)malloc(sizeof(PowerLimiting));

    powerlimiting->targetPower = targetPower;
    powerlimiting->pid = PID_new(1, 0, 0, 0, 100, 231, 0, 0, 0);

    return powerlimiting;
}

void PowerLimiting_limitPower(PowerLimiting* pl, MotorController* mcm, TorqueEncoder* tps){
    pl->pid->controllerMaxima = (tps->travelPercent * 231);
    float processVariable = MCM_getDCCurrent(mcm);
    float targetValue = pl->targetPower / MCM_getDCVoltage(mcm);
    float outputVariable = MCM_getCommandedTorque(mcm);
    float torqueRequest = PID_computeOutput(pl->pid, targetValue, processVariable, outputVariable);

    sbyte2 torqueDeciNM = torqueRequest * 10;
    MCM_set_powerlimitingTorqueCommand(mcm,torqueDeciNM);
}
