#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef PID_H
#define PID_H

typedef struct {
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float setpoint; //Reference,Target value
    float prev_error; // Previous error
    float total_error; // total error 
    float dt;//basically the time interval of each sesnsor value this is in a 
    // dt will be a seperate param in method 
} PID_Controller;

PID_Controller* PID_Init(float kp, float ki, float kd, float setpoint);
void PID_Setpoint_Update(PID_Controller *pid, float setpoint);
void PID_Dt_Update(PID_Controller *pid, float new_dt);
float PID_Compute(PID_Controller *pid, float sensorVal);


#endif