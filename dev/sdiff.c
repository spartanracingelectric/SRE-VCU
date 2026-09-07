/*****************************************************************************
 * sdiff.c - Software Differential Revision (Pre-Torque Vectoring)
 * Initial Author: Andy Van, Sellab Ahmadzai, Akash Karthik
 ******************************************************************************
 * Removes torque from inhub motors to mimic a differential
 *
 * R&D car for now, read update/revision notes at the end of file
 ****************************************************************************/
#include "sdiff.h"
#include <math.h>
#include <stdlib.h>


/*
 * UNITS
 * -----
 * steering_deg  : steering WHEEL angle, degrees, as returned by steering_degrees()
 *                 (+/- 90 deg with the current SAS calibration in sensorCalculations.c)
 * delta         : road WHEEL (rack) angle = steering_deg / STEERING_RATIO, DEGREES.
 * DELTA_MAX,
 * DEADBAND      : also road wheel DEGREES, so they are directly comparable to delta.
 *
 * Everything angular below is in degrees of road wheel angle. Keep it that way -
 * mixing radians in here is what made the deadband unreachable before.
 */
#define DEG_TO_RAD 0.01745329252f  // unused in rev 1, kept for the rev 2 IMU work
// TODO: tune/define these, and set init values when driving and measuring PLS DONT FLASH THIS :(
#define STEERING_RATIO 5.4 //wheel to rack | vaule comes from dylan and the team
#define DELTA_MAX 2 // max road wheel angle, DEGREES | vaule comes from dylan and the team
#define K_DERATE 0 // how aggro torque will decrease with steering angle 
#define K_INNER 0.42 // extra derate on inner (unloaded) wheel 
#define F_MIN 0.5  // minimum torque floor, will cap the baseline torque reduction, 0 < for 
#define DEADBAND 1.0
#define RATE 1.0f // max speed on how low the torques can drop for each motor 
#define SDIFF_LOOP_PERIOD 0.01f // vcu cycle time



static float4 clampf(float4 v, float4 lo, float4 hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static float4 slew(float4 cur, float4 tgt, float4 ratePerSec) {
  float4 step = ratePerSec * SDIFF_LOOP_PERIOD;
  if (tgt > cur)
    return (cur + step < tgt) ? cur + step : tgt;
  if (tgt < cur)
    return (cur - step > tgt) ? cur - step : tgt;
  return cur;
}

SDiff *SDiff_new(bool sdiffToggle) {
  SDiff *me = (SDiff *)malloc(sizeof(SDiff));
  me->sdiffToggle = sdiffToggle;
  me->f_applied = 1.0f;
  me->g_left_appl = 1.0f;
  me->g_right_appl = 1.0f;
  return me;
}

SDiff_Command s_diff_control(SDiff *me, float4 steering_deg,
                             float4 t_driver) {
  SDiff_Command cmd;
  float4 delta = steering_deg / STEERING_RATIO;
  float4 delta_norm = clampf(fabsf(delta) / DELTA_MAX, 0.0f, 1.0f); // clamp norm value for steering ang

  // friction budget
  float4 f = clampf(1.0f - K_DERATE * delta_norm, F_MIN, 1.0f);

  float4 g_in =  1.0f - K_INNER * delta_norm;
  float4 g_out = 1.0f;
  float4 g_left, g_right;
  float4 mult_left, mult_right;

  if (me->sdiffToggle == FALSE) {
    f = 1.0f;
    g_in = 1.0f;
  }

  // left turn
  if (delta > DEADBAND) {
    g_left = g_in;
    g_right = g_out;
  }
  // right turn
  else if (delta < -DEADBAND) {
    g_left = g_out;
    g_right = g_in;
  }
  // straight
  else {
    g_left = 1.0f;
    g_right = 1.0f;
  }

  // slew limit torque
  me->f_applied = slew(me->f_applied, f, RATE);
  me->g_left_appl = slew(me->g_left_appl, g_left, RATE);
  me->g_right_appl = slew(me->g_right_appl, g_right, RATE);


  mult_left  = clampf(me->f_applied * me->g_left_appl,  0.0f, 1.0f);
  mult_right = clampf(me->f_applied * me->g_right_appl, 0.0f, 1.0f);

  cmd.left  = (sbyte4)(t_driver * mult_left);
  cmd.right = (sbyte4)(t_driver * mult_right);

  return cmd;
}

/** GETTER FUNCTIONS **/
bool   SDiff_getToggle(SDiff *me)            { return me->sdiffToggle; }
float4 SDiff_getSharedMultiplier(SDiff *me)  { return me->f_applied;   }
float4 SDiff_getLeftMultiplier(SDiff *me)    { return me->g_left_appl; }
float4 SDiff_getRightMultiplier(SDiff *me)   { return me->g_right_appl;}
