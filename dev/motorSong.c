/*****************************************************************************
 * motorSong.c - Drivetrain warm-up song (VESC/custom-inverter edition)
 * Initial Author: Connor Petri
 ******************************************************************************
 * Speed-based tone engine: on this drivetrain the audible pitch comes from
 * gear-mesh/motor whine, which tracks motor speed, which tracks duty
 * (13:1 gearbox per motor - the motor screams while the wheel turns 13x
 * slower). So each note is a held duty level, pitch proportional to duty,
 * and both rear motors sing in unison. Bench finding: tones are most
 * audible between 15% and 40% duty.
 *
 * Transitions are slew-rate limited, and the song begins with a slow
 * spin-up ramp to the first note - jumping duty from a standstill (no
 * back-EMF) spikes current the external supply can't source. Downward
 * slews are limited too, since a duty drop regens current back into the
 * supply. The song ends (or is skipped/canceled) through a ramp-down for
 * the same reason.
 *
 * Frames use the same extended-frame SET_DUTY messages the MVP ramp uses
 * (0x01 = RL, 0x00 = RR, 4-byte big-endian), sent via
 * CanManager_sendImmediate every update tick.
 ****************************************************************************/

#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "motorSong.h"

//Pitch-to-duty mapping: duty = freq_Hz * MOTORSONG_DUTY_PER_HZ.
//Motor speed is proportional to duty and perceived pitch is proportional
//to speed, so equal musical intervals are equal duty ratios. 110 puts the
//song's core (C3 131Hz -> G4 392Hz) at ~14.4% -> ~43.1% duty, hugging the
//15-40% sweet band; the G4 climax pokes ~3% above it and the low timpani
//G2 sits at ~10.8% (a quieter thump - raise this constant to transpose
//the whole song up if the low notes are inaudible). TUNE BY EAR: this
//constant is the tuning peg; absolute pitch depends on motor kV, pack
//voltage, and the 13:1 mesh, none of which the VCU knows.
#define MOTORSONG_DUTY_PER_HZ       110

//Hard ceiling on any note's duty - the external supply cap (see
//DUTY_CYCLE_MAX in powertrainControl.c)
#define MOTORSONG_DUTY_CEILING      60000

//Spin-up slew from standstill to the first note: ~15 units/ms reaches C3
//(~14400) in about 1 second. Standstill has no back-EMF, so this is the
//gentlest, highest-current-risk transition - keep it slow.
#define MOTORSONG_RAMP_IN_PER_MS    15

//Note-to-note (and ramp-down) slew: the motor is already spinning, so
//transitions can be quicker. 150 units/ms covers the song's widest
//interval (~22000 units) in ~150ms - a short glide into each note.
#define MOTORSONG_NOTE_SLEW_PER_MS  150

//How often duty is updated and frames are sent
#define MOTORSONG_UPDATE_PERIOD_US  5000

//Driver pressing the throttle past this skips the song.
#define MOTORSONG_SKIP_TPS_PERCENT  0.10

//Extended-frame CAN IDs, matching canOutput_sendDebugMessage1
#define MOTORSONG_CAN_ID_REAR_LEFT  0x01
#define MOTORSONG_CAN_ID_REAR_RIGHT 0x00

typedef enum {
    SONG_IDLE = 0,
    SONG_RAMP_IN = 1,   //spinning up to the first note
    SONG_NOTES = 2,     //playing the table
    SONG_RAMP_OUT = 3,  //spinning down after the last note (or skip/cancel)
    SONG_FINISHED = 4
} SongState;

typedef struct {
    ubyte2 freq_Hz;     //0 = hold the current pitch (rests can't be silent
                        //in speed mode - the motor is still spinning)
    ubyte2 duration_ms;
} SongNote;

//Also sprach Zarathustra - the full Sunrise intro (the 2001 fanfare),
//down one octave. Three statements of the rising C-G-C fanfare; the
//answering hits: statement 1 falls major-to-minor (E-Eb), statement 2
//inverts it (Eb-E), statement 3 rises through E-F into the held G climax.
//Timpani C-G alternation between statements. In speed mode every note is
//a duty level and transitions glide, so this plays as a siren-rendition -
//rests hold the previous pitch and just shape the timing.
//Total ~16s - the driver can throttle-skip it. Edit freely - this table
//is the whole song.
static const SongNote song[] = {
    //Statement 1: C G C, answered major falling to minor
    { 131, 700 },                    //C3
    {   0, 120 },
    { 196, 700 },                    //G3
    {   0, 120 },
    { 262, 900 },                    //C4
    {   0,  80 },
    { 330, 300 },                    //E4
    { 311, 500 },                    //Eb4
    {   0, 200 },
    //Timpani: C G C G
    { 131, 180 }, {  98, 180 },
    { 131, 180 }, {  98, 180 },
    {   0, 250 },
    //Statement 2: C G C, answered minor rising to major
    { 131, 700 },
    {   0, 120 },
    { 196, 700 },
    {   0, 120 },
    { 262, 900 },
    {   0,  80 },
    { 311, 300 },                    //Eb4
    { 330, 500 },                    //E4
    {   0, 200 },
    //Timpani: C G C G
    { 131, 180 }, {  98, 180 },
    { 131, 180 }, {  98, 180 },
    {   0, 250 },
    //Statement 3: C G C into the full cadence - E F G held, the sunrise
    { 131, 800 },
    {   0, 120 },
    { 196, 800 },
    {   0, 120 },
    { 262, 1000 },
    {   0,  80 },
    { 330, 350 },                    //E4
    { 349, 350 },                    //F4
    { 392, 1400 },                   //G4 - held
    {   0, 200 },
    //Timpani finale: C G C G C G C G
    { 131, 180 }, {  98, 180 },
    { 131, 180 }, {  98, 180 },
    { 131, 180 }, {  98, 180 },
    { 131, 180 }, {  98, 180 },
};
#define MOTORSONG_LENGTH (sizeof(song) / sizeof(song[0]))

static SongState songState = SONG_IDLE;
static bool cancelRequested = FALSE;
static ubyte2 noteIndex = 0;
static sbyte4 currentDuty = 0;
static ubyte4 timestamp_noteStart = 0;
static ubyte4 timestamp_lastUpdate = 0;

static sbyte4 MotorSong_dutyForNote(const SongNote *note)
{
    sbyte4 duty = (sbyte4)note->freq_Hz * MOTORSONG_DUTY_PER_HZ;
    if (duty > MOTORSONG_DUTY_CEILING)
    {
        duty = MOTORSONG_DUTY_CEILING;
    }
    return duty;
}

//Send one rear-motor SET_DUTY frame, mirroring canOutput_sendDebugMessage1:
//extended frame, 4-byte big-endian signed value
static void MotorSong_sendDutyFrame(CanManager *canMan, ubyte4 canId, sbyte4 value)
{
    IO_CAN_DATA_FRAME frame;
    frame.id = canId;
    frame.id_format = IO_CAN_EXT_FRAME;
    frame.length = 4;
    frame.data[0] = (ubyte1)(value >> 24);
    frame.data[1] = (ubyte1)(value >> 16);
    frame.data[2] = (ubyte1)(value >> 8);
    frame.data[3] = (ubyte1)value;
    CanManager_sendImmediate(canMan, CAN1_LOPRI, &frame, 1);
}

//Write the current duty to both rear motors and put it on the bus
static void MotorSong_applyDuty(CanManager *canMan, _Powertrain *powertrain, sbyte4 duty)
{
    powertrain->motor[2]->dutyCycle_send = duty;
    powertrain->motor[3]->dutyCycle_send = duty;
    MotorSong_sendDutyFrame(canMan, MOTORSONG_CAN_ID_REAR_LEFT, duty);
    MotorSong_sendDutyFrame(canMan, MOTORSONG_CAN_ID_REAR_RIGHT, duty);
}

//Move currentDuty toward target, at most slewPerMs units per elapsed ms.
//Returns TRUE when the target has been reached.
static bool MotorSong_slewToward(sbyte4 target, sbyte4 slewPerMs, ubyte4 elapsed_us)
{
    sbyte4 step = slewPerMs * (sbyte4)(elapsed_us / 1000);
    if (step < slewPerMs)
    {
        step = slewPerMs; //always make progress even on a short tick
    }

    if (currentDuty < target)
    {
        currentDuty += step;
        if (currentDuty >= target) { currentDuty = target; }
    }
    else if (currentDuty > target)
    {
        currentDuty -= step;
        if (currentDuty <= target) { currentDuty = target; }
    }
    return (currentDuty == target);
}

void MotorSong_start(void)
{
    noteIndex = 0;
    currentDuty = 0;
    cancelRequested = FALSE;
    songState = SONG_RAMP_IN;
    IO_RTC_StartTime(&timestamp_lastUpdate);
    IO_RTC_StartTime(&timestamp_noteStart);
}

void MotorSong_cancel(void)
{
    if (songState == SONG_RAMP_IN || songState == SONG_NOTES)
    {
        cancelRequested = TRUE;
    }
}

void MotorSong_skip(CanManager *canMan, _Powertrain *powertrain)
{
    //Skip still rides the ramp-down - an instant duty drop regens hard
    //into the supply
    if (songState == SONG_RAMP_IN || songState == SONG_NOTES)
    {
        songState = SONG_RAMP_OUT;
    }
}

void MotorSong_reset(void)
{
    if (songState == SONG_FINISHED)
    {
        songState = SONG_IDLE;
    }
}

bool MotorSong_isPlaying(void)
{
    return (songState == SONG_RAMP_IN || songState == SONG_NOTES || songState == SONG_RAMP_OUT);
}

bool MotorSong_hasFinished(void)
{
    return (songState == SONG_FINISHED);
}

void MotorSong_safetyUpdate(CanManager *canMan, _Powertrain *powertrain, TorqueEncoder *tps)
{
    if (MotorSong_isPlaying() == FALSE)
    {
        return;
    }

    //Driver on the throttle = they want the ramp, not the concert
    if (tps->calibrated == TRUE && tps->travelPercent > MOTORSONG_SKIP_TPS_PERCENT)
    {
        MotorSong_skip(canMan, powertrain);
    }
}

void MotorSong_fastTask(CanManager *canMan, _Powertrain *powertrain)
{
    if (MotorSong_isPlaying() == FALSE)
    {
        cancelRequested = FALSE;
        return;
    }

    if (cancelRequested == TRUE)
    {
        cancelRequested = FALSE;
        songState = SONG_RAMP_OUT;
    }

    ubyte4 elapsed_us = IO_RTC_GetTimeUS(timestamp_lastUpdate);
    if (elapsed_us < MOTORSONG_UPDATE_PERIOD_US)
    {
        return;
    }
    IO_RTC_StartTime(&timestamp_lastUpdate);

    switch (songState)
    {
        case SONG_RAMP_IN:
            //Gentle spin-up from standstill to the first note
            if (MotorSong_slewToward(MotorSong_dutyForNote(&song[0]), MOTORSONG_RAMP_IN_PER_MS, elapsed_us) == TRUE)
            {
                songState = SONG_NOTES;
                noteIndex = 0;
                IO_RTC_StartTime(&timestamp_noteStart);
            }
        break;

        case SONG_NOTES:
            if (IO_RTC_GetTimeUS(timestamp_noteStart) >= (ubyte4)song[noteIndex].duration_ms * 1000)
            {
                noteIndex++;
                IO_RTC_StartTime(&timestamp_noteStart);
                if (noteIndex >= MOTORSONG_LENGTH)
                {
                    songState = SONG_RAMP_OUT;
                    break;
                }
            }
            //Rests (freq 0) hold the current pitch; real notes glide to
            //their duty at the note slew rate
            if (song[noteIndex].freq_Hz != 0)
            {
                MotorSong_slewToward(MotorSong_dutyForNote(&song[noteIndex]), MOTORSONG_NOTE_SLEW_PER_MS, elapsed_us);
            }
        break;

        case SONG_RAMP_OUT:
            //Controlled spin-down - limits regen current into the supply
            if (MotorSong_slewToward(0, MOTORSONG_NOTE_SLEW_PER_MS, elapsed_us) == TRUE)
            {
                songState = SONG_FINISHED;
            }
        break;

        default:
        break;
    }

    MotorSong_applyDuty(canMan, powertrain, currentDuty);
}
