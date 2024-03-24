#include <stdio.h>
#include "pid.h"
// VCU will run this once, outside of the while loop 
PID_Controller* PID_Init(float kp, float ki, float kd, float setpoint)
{
    // pid will be replaced with me
    PID_Controller* pid = (PID_Controller*)malloc(sizeof(PID_Controller));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint; 
    pid->prev_error=0.0;
    pid->total_error=0.0;
    pid->dt=0.0;

    return pid;
}
// Within the while loop in VCU 
void PID_Setpoint_Update(PID_Controller *pid, float setpoint)
{
    pid->setpoint = setpoint; 
}
void PID_Dt_Update(PID_Controller *pid, float new_dt)
{
    pid->dt = new_dt;
}
float PID_Compute(PID_Controller *pid, float sensorVal)
{
    float error = pid->setpoint - sensorVal; 
    float proportional = pid->kp*error; 
    float integral = pid->ki * (pid->total_error + error)* pid->dt;
    float derivative =  pid->kd * (error - pid->prev_error)/ pid->dt ;

    float output = proportional + integral + derivative;
    pid->prev_error = error;
    pid->total_error += error* pid->dt; 

    return output; 
}

/*

int main()
{
    float kp = 0.5;
    float ki = 0.2;
    float kd = 0.1;
    float setpoint = 4.6;
    PID_Controller* pid = PID_Init(kp, ki, kd, setpoint);
    PID_Dt_Update(pid, 0.1);
    
    float sensorVal = 4.8;

    float output = PID_Compute(pid,sensorVal);


    printf("%f",output);
    return 0;
}
*/

