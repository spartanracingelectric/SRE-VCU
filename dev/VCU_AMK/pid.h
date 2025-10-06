//"Include guard" - prevents this file from being #included more than once
#ifndef PID_H
#define PID_H

#include "IO_Driver.h"
typedef enum { Kp,Ti,Td,setpoint,totalError,controllerMaxima, controllerMinima,cleggIntegrator, Kb } PID_Settings;

typedef struct {
    float Kp; // Proportional gain
    float Ti; // Integral Time (from https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Standard_versus_parallel_(ideal)_form) The integral component adjusts the error value to compensate for the sum of all past errors, with the intention of completely eliminating them in Ti seconds (or samples)
    float Td; // Derivative Time (from https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Standard_versus_parallel_(ideal)_form) The derivative components term attempts to predict the error value at Td seconds (or samples) in the future, assuming that the loop control remains unchanged. 
    float setpoint;
    float error;
    float previousError;
    float totalError;
    float dt;
    float processVariable; // current output of process being controlled 
    float controlVariable; // adjustment to process desired PID before Anti-Windup
    float output; // value to be sent to process Controller
    float controllerMaxima; // maximum output of process being controlled (can be user defined or datasheet spec'd)
    float controllerMinima; // minimum output of process being controlled (can be user defined or datasheet spec'd)
    ubyte1 clegg; // (Value: 0 or 1 (Used in Boolean Logic)) Google "clegg integrator" for more explanation on what this is doing
    float Kb; // back calculation gain (default should be: 0 if not wanting to use)
} PID_Controller;

PID_Controller* PID_new(float Kp, float Ti, float Td, float setpoint, float dt, float controllerMaxima, float controllerMinima, float Kb, ubyte1 clegg);
void PID_updateSettings(PID_Controller* pid, PID_Settings setting, float input1);
float PID_computeControlVariable(PID_Controller *pid, float processVariable);
float PID_computeOutput(PID_Controller *pid, float targetValue, float processVariable, float outputVariable);

#endif