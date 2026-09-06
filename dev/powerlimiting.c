#include <stdlib.h>
#include <math.h>
#include "powerlimiting.h"

PowerLimiting* PowerLimiting_new(float targetPower){
    PowerLimiting* powerlimiting = (PowerLimiting*)malloc(sizeof(PowerLimiting));

    powerlimiting->targetPower = targetPower;
    powerlimiting->pid = PID_new(1, 0, 0, 0, 100, 231, 0, 0, 0);

    return powerlimiting;
}
