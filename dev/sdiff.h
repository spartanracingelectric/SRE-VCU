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



typedef struct _SDiff {
  bool sdiffToggle;    // feature on/off switch, set at construction
  float4 f_applied;    // smoothed shared friction multiplier
  float4 g_left_appl;  // smoothed left
  float4 g_right_appl; // smoothed right
} SDiff;

/** CONSTRUCTOR **/
SDiff *SDiff_new(bool sdiffToggle);
typedef struct _SDiff_Command {
  sbyte4 left;  // torque request 
  sbyte4 right; // torque request
} SDiff_Command;

SDiff_Command s_diff_control(SDiff *me, float4 steering_deg, float4 t_driver);

/** GETTER FUNCTIONS **/
bool   SDiff_getToggle(SDiff *me);
float4 SDiff_getSharedMultiplier(SDiff *me);
float4 SDiff_getLeftMultiplier(SDiff *me);
float4 SDiff_getRightMultiplier(SDiff *me);

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
