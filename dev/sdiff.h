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

#define DEG_TO_RAD 0.01745329252f

// TODO: tune/define these, and set init values when driving and measuring PLS DONT FLASH THIS :(
#define STEERING_RATIO 5.4 //wheel to rack | vaule comes from dylan and the team
#define DELTA_MAX 2 // max steering angle | vaule comes from dylan and the team
#define K_DERATE 0.25 // how aggro torque will decrease with steering angle 
/*
The total derate is 1 - K_DERATE * delta_norm, so if K_DERATE = 0.25 and delta_norm = 1, then the total derate is 0.75, meaning that the torque will be reduced by 25% at max steering angle.
*/
#define K_INNER 0.35 // extra derate on inner (unloaded) wheel 
//inner wheel will be derated by K_INNER * delta_norm, so if K_INNER = 0.35 and delta_norm = 1, then the inner wheel will be derated by 35% at max steering angle.

#define F_MIN 0.5  // minimum torque floor, will cap the baseline torque reduction, 0 < for 

/*
pure safety, 1 < for more aggressive torque vectoring 

if F_MIN = 0.5, then the torque will never be reduced below 50% of the original torque, meaning that the torque will be reduced by at most 50% at max steering angle.

regen lowk try capping at 0 first then negative later irl
*/

#define DEADBAND 1.0   // tolerance around center steering position 
//if the steering angle is within DEADBAND degrees of center, then the torque will not be reduced, meaning that the torque will be reduced by 0% at center steering angle.


#define RATE 1.0f // max speed on how low the torques can drop for each motor 

/*
slew rate of the torque multipliers, in multiplier units per second (NOT percent)
step per 10ms loop = RATE * SDIFF_LOOP_PERIOD = 0.01, so at full lock the shared factor
reaches 0.75 in 0.25s, the inner wheel reaches 0.65 in 0.35s, and the F_MIN floor of 0.5 in 0.5s.
Applies in both directions, so torque comes back at the same rate when straightening out.
*/

#define SDIFF_LOOP_PERIOD 0.01f // 10ms off main

static inline float4 clampf(float4 v, float4 lo, float4 hi);
static inline float4 slew(float4 cur, float4 tgt, float4 ratePerSec);

// running state of SDiff
typedef struct _SDiff {
  float4 f_applied;    // smoothed shared friction multiplier
  float4 g_left_appl;  // smoothed left
  float4 g_right_appl; // smoothed right
} SDiff;

SDiff *SDiff_new(void);

// TODO: see what the custom inverters use idk any docs on that ngl
// AMK CAN protocol takes 16 bit signed field on bus(not using)
typedef struct _SDiff_Command {
  sbyte2 left;  // torque request 
  sbyte2 right; // torque request
} SDiff_Command;

SDiff_Command s_diff_control(SDiff *me, float4 steering_deg, float4 t_driver);

#endif

/*
 * S_diff Revision 1:
 * Sensors: Steering angle Only
 * Primitive open loop system that will derate based off steering angle
 * logic: define what side the car is turning on, on that side reduce torque to
 * the wheels on that side
 *
 * S_diff Revision 2(TODO):
 * Sensors, Steering angle, 6axis IMU
 *
 */
