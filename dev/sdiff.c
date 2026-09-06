/*****************************************************************************
 * sdiff.c - Software Differential Revision(Pre-Torque Vectoring)
 * Initial Author: Andy Van, Sellab Ahmadzai, Akash Karthik
 ******************************************************************************
 * Removes torque from inhub motors to mimic a differential
 *
 * R&D car for now, read update/revision notes at the end of file
 ****************************************************************************/
#include "sdiff.h"
#include <math.h>
#include <stdlib.h>

static inline float4 clampf(float4 v, float4 lo, float4 hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static inline float4 slew(float4 cur, float4 tgt, float4 ratePerSec) {
  float4 step = ratePerSec * SDIFF_LOOP_PERIOD;
  if (tgt > cur)
    return (cur + step < tgt) ? cur + step : tgt;
  if (tgt < cur)
    return (cur - step > tgt) ? cur - step : tgt;
  return cur;
}

SDiff *SDiff_new(void) {
  SDiff *me = (SDiff *)malloc(sizeof(SDiff));
  me->f_applied = 1.0f;
  me->g_left_appl = 1.0f;
  me->g_right_appl = 1.0f;
  return me;
}

SDiff_Command s_diff_control(SDiff *me, float4 steering_deg,
                             float4 t_driver) {
  SDiff_Command cmd;

  float4 delta = (steering_deg * DEG_TO_RAD) / STEERING_RATIO;
  float4 delta_norm = clampf(fabsf(delta) / DELTA_MAX, 0.0f, 1.0f); // clamp norm value for steering ang

  // friction budget
  float4 f = clampf(1.0f - K_DERATE * delta_norm, F_MIN, 1.0f);

  float4 g_in =  1.0f - K_INNER * delta_norm;
  float4 g_out = 1.0f;
  float4 g_left, g_right;

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

  cmd.left  = (sbyte2)clampf(t_driver * me->f_applied * me->g_left_appl, -2310.0f, 2310.0f);
  cmd.right = (sbyte2)clampf(t_driver * me->f_applied * me->g_right_appl, -2310.0f, 2310.0f);

  return cmd;
}
