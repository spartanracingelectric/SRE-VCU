/*****************************************************************************
 * motorSong.h - Drivetrain warm-up song
 * Initial Author: Connor Petri
 ******************************************************************************
 * Final stage of the RTD warm-up cycle: plays a note table through the AMK
 * motors by toggling small zero-mean torque requests at audio frequency.
 * The powertrain state machine holds in WARMUP_SONG until the song finishes.
 ****************************************************************************/

#ifndef _MOTORSONG_H
#define _MOTORSONG_H

#include "IO_Driver.h"
#include "canManager.h"
#include "torqueEncoder.h"
#include "sensors.h"

//Begin playing the song. Caller must guarantee all four inverters are on and
//torque limits are already set (the warm-up handshake stages have finished).
void MotorSong_start(_Powertrain *powertrain);

//Abort: silence all motors, restore pre-song torque limits, push zeroed
//setpoint frames onto the bus. Song replays on the next warm-up attempt.
void MotorSong_stop(CanManager *canMan, _Powertrain *powertrain);

//Driver skip: same cleanup as stop, but the song counts as finished so the
//warm-up stage completes immediately.
void MotorSong_skip(CanManager *canMan, _Powertrain *powertrain);

//Re-arm the song after a fault knocked the powertrain out of RTD, so the
//next warm-up cycle plays it again (mirrors rtdsPlayed = FALSE).
void MotorSong_reset(void);

bool MotorSong_isPlaying(void);
bool MotorSong_hasFinished(void);

//Call once per 10ms main loop cycle. Skips on driver throttle input; aborts
//on HVIL open, inverter error, or an inverter dropping out.
void MotorSong_safetyUpdate(CanManager *canMan, _Powertrain *powertrain, TorqueEncoder *tps, Sensor *HVILTermSense);

//Call as often as possible (from the end-of-loop wait) - generates the tone
//by toggling torque requests and sending setpoint frames directly, bypassing
//CanManager_send's change-detection/rate-limit history.
void MotorSong_fastTask(CanManager *canMan, _Powertrain *powertrain);

#endif //_MOTORSONG_H
