/*****************************************************************************
 * sdiff.h - Software Differential Revision(Pre-Torque Vectoring)
 * Initial Author: Andy Van, Sellab Ahmadzai
 ******************************************************************************
 * Removes torque from inhub motors to mimic a differential
 *
 * R&D car for now, read update/revision notes at the end of file
 ****************************************************************************/

#ifndef _SDIFF_H
#define _SDIFF_H

#include "IO_Driver.h"

// TODO: tune/define these, and set init values when driving and measuring
#define K_DERATE 0 // how aggro torque will decrease with steering angle
#define F_MIN 0 // minimum torque floor, will cap the baseline torque reduction, 0 < for regen                   
                // lowk try capping at 0 first then negative later irl

#define DEADBAND 0 // tolerance around center steering position
#define MAX_DERATE 0 // max speed on how low the torques can drop for each motor
#define SDIFF_LOOP_PERIOD 0.01f // 10ms off main


static inline float4 clampf(float4 v, float4 lo, float4 hi);
static inline float4 slew(float4 cur, float4 tgt, float4 ratePerSec);


// running state of SDiff
typedef struct _SDiff {
    float4 f_applied;    // smoothed shared friction multiplier 
    float4 g_left_appl;  // smoothed left 
    float4 g_right_appl; // smoothed right 
} SDiff;


// AMK CAN protocol takes 16 bit signed field on bus
typedef struct _SDiff_Command {
    sbyte2 left;   /* rear-left  torque request */
    sbyte2 right;  /* rear-right torque request */
} SDiff_Command;



#endif

/*
 * S_diff Revision 1:
 * Sensors: Steering angle Only
 * Primitve open loop system that will derate based off steering angle
 * logic: define what side the car is turning on, on that side reduce torque to the wheels on that side
 *
 * S_diff Revision 2(TODO):
 * Sensors, Steering angle, 6axis IMU
 */
