/*****************************************************************************
 * motorSong.h - Drivetrain warm-up song (VESC/custom-inverter edition)
 * Initial Author: Connor Petri
 ******************************************************************************
 * The warm-up cycle for the duty-cycle rig: when the Eco button activates
 * the MVP ramp, the song plays through the rear motors first, and the ramp
 * only engages once it finishes. Tones are made by toggling a small
 * zero-mean command value at the note's frequency.
 ****************************************************************************/

#ifndef _MOTORSONG_H
#define _MOTORSONG_H

#include "IO_Driver.h"
#include "canManager.h"
#include "torqueEncoder.h"
#include "sensors.h"

//Begin playing the song (called when the MVP ramp is activated).
void MotorSong_start(void);

//Request an abort (e.g. ramp toggled off mid-song). Processed by the fast
//task, which silences both motors; the song replays on the next activation.
void MotorSong_cancel(void);

//Driver skip: silence both motors and count the song as finished so the
//ramp can engage immediately.
void MotorSong_skip(CanManager *canMan, _Powertrain *powertrain);

//Re-arm a finished song so the next ramp activation plays it again.
void MotorSong_reset(void);

bool MotorSong_isPlaying(void);
bool MotorSong_hasFinished(void);

//Call once per 10ms main loop cycle. Skips the song on driver throttle
//input (the MVP rig intentionally has no other gating - parity with the
//duty ramp itself).
void MotorSong_safetyUpdate(CanManager *canMan, _Powertrain *powertrain, TorqueEncoder *tps);

//Call as often as possible (from the end-of-loop wait) - generates the
//tones by toggling command values and sending frames directly.
void MotorSong_fastTask(CanManager *canMan, _Powertrain *powertrain);

#endif //_MOTORSONG_H
