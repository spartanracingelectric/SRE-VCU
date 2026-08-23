/*****************************************************************************
 * motorSong.c - Drivetrain warm-up song (VESC/custom-inverter edition)
 * Initial Author: Connor Petri
 ******************************************************************************
 * Plays a note table through the rear motors by toggling a small zero-mean
 * command value at the note's frequency, using the same extended-frame
 * command messages the MVP duty ramp uses (0x101 = rear left, 0x100 = rear
 * right, 4-byte big-endian value). Net command over a period is zero, so
 * the motors sing without spinning up.
 *
 * Frames go out via CanManager_sendImmediate at toggle edges (audio rate);
 * the 10ms canOutput_sendDebugMessage1 stream stays consistent because the
 * toggled value is also written into dutyCycle_send.
 ****************************************************************************/

#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "motorSong.h"

//Peak command value while voicing a note, in the same raw units as the MVP
//duty ramp (DUTY_CYCLE_MAX = 100000), i.e. 5% of the ramp's maximum.
//BENCH-TUNE THIS: if the motors are silent, raise it; if they twitch or
//get warm fast, lower it.
#define MOTORSONG_AMPLITUDE 5000

//Silent tail on every note so repeated notes articulate instead of
//slurring. Must be shorter than the shortest note in the table.
#define MOTORSONG_NOTE_GAP_US 40000

//Driver pressing the throttle past this skips the song.
#define MOTORSONG_SKIP_TPS_PERCENT 0.10

//Extended-frame CAN IDs for the rear motor command messages, matching
//canOutput_sendDebugMessage1
#define MOTORSONG_CAN_ID_REAR_LEFT  0x101
#define MOTORSONG_CAN_ID_REAR_RIGHT 0x100

typedef enum {
    SONG_IDLE = 0,
    SONG_PLAYING = 1,
    SONG_FINISHED = 2
} SongState;

typedef struct {
    ubyte2 freq_Hz;     //0 = rest
    ubyte2 duration_ms;
    ubyte1 motorIndex;  //voicing motor by powertrain index; only the rears
                        //exist on this rig, so 0/2 voice RL and 1/3 voice RR
} SongNote;

//Also sprach Zarathustra - the full Sunrise intro (the 2001 fanfare),
//down one octave. Three statements of the rising C-G-C fanfare; the
//answering hits: statement 1 falls major-to-minor (E-Eb), statement 2
//inverts it (Eb-E), statement 3 rises through E-F into the held G climax.
//Timpani C-G pounding alternates across the rear pair between statements.
//Total ~16s - the driver can throttle-skip it. Edit freely - this table
//is the whole song.
static const SongNote song[] = {
    //Statement 1: C G C, answered major falling to minor
    { 131, 700, 0 },                 //C3
    {   0, 120, 0 },
    { 196, 700, 1 },                 //G3
    {   0, 120, 0 },
    { 262, 900, 2 },                 //C4
    {   0,  80, 0 },
    { 330, 300, 3 },                 //E4
    { 311, 500, 3 },                 //Eb4
    {   0, 200, 0 },
    //Timpani: C G C G
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
    {   0, 250, 0 },
    //Statement 2: C G C, answered minor rising to major
    { 131, 700, 0 },
    {   0, 120, 0 },
    { 196, 700, 1 },
    {   0, 120, 0 },
    { 262, 900, 2 },
    {   0,  80, 0 },
    { 311, 300, 3 },                 //Eb4
    { 330, 500, 3 },                 //E4
    {   0, 200, 0 },
    //Timpani: C G C G
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
    {   0, 250, 0 },
    //Statement 3: C G C into the full cadence - E F G held, the sunrise
    { 131, 800, 0 },
    {   0, 120, 0 },
    { 196, 800, 1 },
    {   0, 120, 0 },
    { 262, 1000, 2 },
    {   0,  80, 0 },
    { 330, 350, 3 },                 //E4
    { 349, 350, 0 },                 //F4
    { 392, 1400, 1 },                //G4 - held
    {   0, 200, 0 },
    //Timpani finale: C G C G C G C G
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
};
#define MOTORSONG_LENGTH (sizeof(song) / sizeof(song[0]))

static SongState songState = SONG_IDLE;
static bool cancelRequested = FALSE;
static ubyte2 noteIndex = 0;
static ubyte4 timestamp_noteStart = 0;
static ubyte4 timestamp_lastToggle = 0;
static sbyte1 toggleSign = 1;
static bool noteSilenced = FALSE;

//Powertrain motor index for a note: 0/2 -> RL (motor[2]), 1/3 -> RR (motor[3])
static ubyte1 MotorSong_rearIndexForNote(const SongNote *note)
{
    return ((note->motorIndex % 2) == 0) ? 2 : 3;
}

//Send one rear-motor command frame, mirroring canOutput_sendDebugMessage1:
//extended frame, 4-byte big-endian signed value
static void MotorSong_sendCommandFrame(CanManager *canMan, ubyte1 rearIndex, sbyte4 value)
{
    IO_CAN_DATA_FRAME frame;
    frame.id = (rearIndex == 2) ? MOTORSONG_CAN_ID_REAR_LEFT : MOTORSONG_CAN_ID_REAR_RIGHT;
    frame.id_format = IO_CAN_EXT_FRAME;
    frame.length = 4;
    frame.data[0] = (ubyte1)(value >> 24);
    frame.data[1] = (ubyte1)(value >> 16);
    frame.data[2] = (ubyte1)(value >> 8);
    frame.data[3] = (ubyte1)value;
    CanManager_sendImmediate(canMan, CAN1_LOPRI, &frame, 1);
}

static void MotorSong_silenceAll(CanManager *canMan, _Powertrain *powertrain)
{
    for(ubyte1 i = 2; i < 4; ++i){
        powertrain->motor[i]->dutyCycle_send = 0;
        MotorSong_sendCommandFrame(canMan, i, 0);
    }
}

void MotorSong_start(void)
{
    noteIndex = 0;
    toggleSign = 1;
    noteSilenced = FALSE;
    cancelRequested = FALSE;
    songState = SONG_PLAYING;
    IO_RTC_StartTime(&timestamp_noteStart);
    IO_RTC_StartTime(&timestamp_lastToggle);
}

void MotorSong_cancel(void)
{
    if(songState == SONG_PLAYING){
        cancelRequested = TRUE;
    }
}

void MotorSong_skip(CanManager *canMan, _Powertrain *powertrain)
{
    if(songState != SONG_PLAYING){
        return;
    }
    MotorSong_silenceAll(canMan, powertrain);
    songState = SONG_FINISHED;
}

void MotorSong_reset(void)
{
    if(songState == SONG_FINISHED){
        songState = SONG_IDLE;
    }
}

bool MotorSong_isPlaying(void)
{
    return (songState == SONG_PLAYING);
}

bool MotorSong_hasFinished(void)
{
    return (songState == SONG_FINISHED);
}

void MotorSong_safetyUpdate(CanManager *canMan, _Powertrain *powertrain, TorqueEncoder *tps)
{
    if(songState != SONG_PLAYING){
        return;
    }

    //Driver on the throttle = they want the ramp, not the concert
    if(tps->calibrated == TRUE && tps->travelPercent > MOTORSONG_SKIP_TPS_PERCENT){
        MotorSong_skip(canMan, powertrain);
    }
}

void MotorSong_fastTask(CanManager *canMan, _Powertrain *powertrain)
{
    if(songState != SONG_PLAYING){
        cancelRequested = FALSE;
        return;
    }

    if(cancelRequested == TRUE){
        MotorSong_silenceAll(canMan, powertrain);
        cancelRequested = FALSE;
        songState = SONG_IDLE;
        return;
    }

    const SongNote *note = &song[noteIndex];
    ubyte1 rearIndex = MotorSong_rearIndexForNote(note);
    _DriveInverter *motor = powertrain->motor[rearIndex];
    ubyte4 noteElapsed_us = IO_RTC_GetTimeUS(timestamp_noteStart);
    ubyte4 noteDuration_us = (ubyte4)note->duration_ms * 1000;

    //Note over - silence its motor and advance
    if(noteElapsed_us >= noteDuration_us){
        if(motor->dutyCycle_send != 0){
            motor->dutyCycle_send = 0;
            MotorSong_sendCommandFrame(canMan, rearIndex, 0);
        }
        noteIndex++;
        if(noteIndex >= MOTORSONG_LENGTH){
            MotorSong_silenceAll(canMan, powertrain);
            songState = SONG_FINISHED;
            return;
        }
        toggleSign = 1;
        noteSilenced = FALSE;
        IO_RTC_StartTime(&timestamp_noteStart);
        IO_RTC_StartTime(&timestamp_lastToggle);
        return;
    }

    //Rest, or the articulation gap at the end of a note
    if(note->freq_Hz == 0 ||
       (noteDuration_us > MOTORSONG_NOTE_GAP_US && noteElapsed_us >= noteDuration_us - MOTORSONG_NOTE_GAP_US))
    {
        if(noteSilenced == FALSE && motor->dutyCycle_send != 0){
            motor->dutyCycle_send = 0;
            MotorSong_sendCommandFrame(canMan, rearIndex, 0);
        }
        noteSilenced = TRUE;
        return;
    }

    //Voice the note: square wave at freq_Hz via alternating command sign
    ubyte4 halfPeriod_us = 500000UL / note->freq_Hz;
    if(IO_RTC_GetTimeUS(timestamp_lastToggle) >= halfPeriod_us){
        IO_RTC_StartTime(&timestamp_lastToggle);
        toggleSign = -toggleSign;
        motor->dutyCycle_send = (sbyte4)toggleSign * MOTORSONG_AMPLITUDE;
        MotorSong_sendCommandFrame(canMan, rearIndex, motor->dutyCycle_send);
    }
}
