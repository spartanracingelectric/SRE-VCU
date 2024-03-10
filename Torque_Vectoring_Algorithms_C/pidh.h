#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
// this PID has to be in an external file from the TV so as the VCU goes through the loop the pointer values get updated accordingly

// Power Distribution is changed front/rear 
//therefore we need different KiKpKd values 
//CONCEPT: front is based on the back torque motor 
//torque based on wheel turning angle (using yaw to detect oversteer and underseer)

float preverror_yaw=0.0;
float totalerror_yaw=0.0;
 float *prevError_yaw= &preverror_yaw;
 float *totalError_yaw= &totalerror_yaw;

 //float *prevError_slipFR;
 //float *totalError_slipFR = NULL;

//extern float *prevError_slipRR = NULL;
//extern float *totalError_slipRR = NULL;

//extern float *prevError_slipFL = NULL;
//extern float *totalError_slipFL = NULL;

//extern float *prevError_slipRL = NULL;
//extern float *totalError_slipRL = NULL;

typedef struct {
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float prev_error; // Previous error
    float total_error; // total error 
    float time_interval;//basically the time interval of each sesnsor value this is in a 
    // 
} PID_Params;
/*
float PID_Controller(float setpoint, float sensorVal, float PID_Params *pid)
{
    float error = setpoint - sensorVal; 
    float proportional = pid->kp*error; 
    float integral = pid->ki * (pid->total_error+ error)*pid->time_interval;
    float derivative =  pid->kd * (error - pid->prev_error);

    float output = proportional + integral + derivative;

}
*/
float PID_controller(float setpoint, float sensorVal, float Kp, float Ki, float Kd, float time_interval, float *totalerror, float *preverror)
{
    float error = setpoint - sensorVal; 
    float proportional = Kp-error; 

    float total_error = *totalerror; 
    float previous_error = *preverror; 

    float integral = Ki * (total_error + error) * time_interval;
    float derivative = Kd * (error - previous_error)/ time_interval; 

    float output = proportional + integral + derivative;

     *preverror = error;
     *totalerror += error; 
     return output; 

}

int main()
{
//Kp_slip = 250;
//Ki_slip = 50;
//Kd_slip = 0;
//Kp_yaw = 100;
//Ki_yaw = 10;
//Kd_yaw = 0;

    float pidoutput = PID_controller(0.2, 0.17,100.0,10.0,0.0,0.1, totalError_yaw, prevError_yaw);

    

    printf("%f",pidoutput);
    return 0;
}


