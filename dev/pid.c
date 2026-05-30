#include <stdlib.h>
#include <math.h>
#include "pid.h"

PID_Controller* PID_new(float Kp, float Ti, float Td, float setpoint, float dt, float controllerMaxima, float controllerMinima, float Kb, ubyte1 clegg){
    PID_Controller* pid = (PID_Controller*)malloc(sizeof(PID_Controller));
    pid->Kp = Kp;
    pid->Ti = Ti;
    pid->Td = Td;
    pid->setpoint = setpoint; 
    pid->previousError = 0.0;
    pid->totalError = 0.0;
    pid->dt = dt;
    pid->processVariable = 0;
    pid->controlVariable = 0;
    pid->output = 0;
    pid->controllerMaxima = controllerMaxima;
    pid->controllerMinima = controllerMinima;
    pid->clegg = clegg;
    pid->Kb = Kb;

    return pid;
}

void PID_updateSettings(PID_Controller* pid, PID_Settings setting, float input1){
    switch(setting)
    {
        case Kp:
        pid->Kp = input1;
        break;

        case Ti:
        pid->Ti = input1;
        break;

        case Td:
        pid->Td = input1;
        break;

        case totalError:
            pid->totalError = input1;
        break;

        case controllerMaxima:
            pid->controllerMaxima = input1;
        break;

        case controllerMinima:
            pid->controllerMinima = input1;
        break;

        case cleggIntegrator:
            pid->clegg = (ubyte1)input1;
        break;

        case Kb:
            pid->Kb = input1;
        break;
    }
}

/** @function 
 * This funciton can be called directly if you want to set up your own feed-forward loop, 
 * or if the intention of the PID is to directly use the output (no adding to output Variable)
 * Notably, you will need to add your own anti-windup guardrails
 * 
 */
float PID_computeControlVariable(PID_Controller *pid, float processVariable){
    pid->error = pid->setpoint - processVariable;
    if((abs(pid->error - pid->previousError) > abs(pid->error) || pid->error == 0 ) && pid->clegg == 1){
        pid->totalError = 0;
    }
    pid->totalError += pid->error * pid->dt; // added before because Ki ( or 1/Ti ) in equ. is multiplied by result of (total error + current error)
    pid->controlVariable = pid->Kp * (pid->error + 1 / pid->Ti * pid->totalError + 1 / pid->Td * (pid->error - pid->previousError) * pid->dt);
    pid->previousError = pid->error;
    return pid->controlVariable;
}
/** @function
 * Designed for all-in-one easy calling, no outsourcing of anti-windup behaviors
 */
float PID_computeOutput(PID_Controller *pid, float targetValue, float processVariable, float outputVariable){
    pid->setpoint = targetValue;
    pid->output = outputVariable + PID_computeControlVariable(pid, processVariable);
    if(pid->output > pid->controllerMaxima){
        pid->totalError -= pid->Kb * (pid->output - pid->controllerMaxima); // back calculating excessive error with Kb gain value (easiest implementation) (see https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)
        pid->output = pid->controllerMaxima;
    }
    if(pid->output < pid->controllerMinima){
        pid->totalError += pid->Kb * (pid->controllerMinima - pid->output); // back calculating excessive error with Kb gain value (easiest implementation), but adding bc raising output to lowest value of process being controlled
        pid->output = pid->controllerMinima;
    }
    return pid->output;
}
