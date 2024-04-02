#include <stdio.h>
#include "pid.h"
#include "yawhashtable.h"
#include "yaw.h"
#include "slip.h"


int main()
{
    HashTable* yaw_hashtable = createHashTable();
    initializeYawHashTable(yaw_hashtable);

    PID_Controller *yaw_Controller = PID_Init(100.0,10.0,0.0,0.2);
    PID_Controller *slip_Controller_FR = PID_Init(250.0,50.0,0.0,0.2);
    PID_Controller *slip_Controller_FL = PID_Init(250.0,50.0,0.0,0.2);
    PID_Controller *slip_Controller_RR = PID_Init(250.0,50.0,0.0,0.2);
    PID_Controller *slip_Controller_RL = PID_Init(250.0,50.0,0.0,0.2);
    while(1)
    {
        PID_Dt_Update(yaw_Controller, 0.15);
        PID_Setpoint_Update(yaw_Controller, getYaw(25.0,20.0));// Updates the setpoint based on yaw lookuptable values defined in yaw.c 
        float PID_yaw_value = PID_Compute(yaw_Controller, 0.4);

        PID_Dt_Update(slip_Controller_FR, 0.15);
        PID_Setpoint_Update(slip_Controller_FR,slipAdjust(0.3));
        float PID_slip_value_FR = PID_Compute(slip_Controller_FR, 0.2);

    }
    // free the storage of PID controller, HashTable etc.. 
    
    
    return 0;

}