/*****************************************************************************
 * motorSong.c - Drivetrain warm-up song
 * Initial Author: Connor Petri
 ******************************************************************************
 * Plays a note table through the AMK motors by toggling a small zero-mean
 * torque request at the note's frequency. One motor voices each note
 * (round-robin FL/FR/RL/RR) so the current draw is spread across the
 * drivetrain. Net torque over a period is zero, and the torque limits are
 * clamped to the song amplitude for the duration, so the car cannot drive
 * off while singing.
 *
 * Frames are sent with CanManager_sendImmediate because CanManager_send's
 * history filter (25ms min interval, last-byte-only change detection) would
 * swallow audio-rate setpoint updates.
 *
 * The audible ceiling is set by how fast the inverters ingest setpoints,
 * which we haven't measured - if high notes come out silent or wrong,
 * transpose the table down an octave (halve the freq_Hz values).
 ****************************************************************************/

#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "motorSong.h"

//Peak torque request while voicing a note, in raw AMK torque units.
//For scale: the drive torque limit set in TORQUE_LIMIT_SET is 210.
#define MOTORSONG_TORQUE_AMPLITUDE 20

//Silent tail on every note so repeated notes articulate instead of slurring.
//Must be shorter than the shortest note duration in the table.
#define MOTORSONG_NOTE_GAP_US 40000

//Abort threshold: driver pressing the throttle past this skips the song.
#define MOTORSONG_SKIP_TPS_PERCENT 0.10

typedef enum {
    SONG_IDLE = 0,
    SONG_PLAYING = 1,
    SONG_FINISHED = 2
} SongState;

typedef struct {
    ubyte2 freq_Hz;     //0 = rest
    ubyte2 duration_ms;
    ubyte1 motorIndex;  //which motor voices the note: 0=FL 1=FR 2=RL 3=RR
} SongNote;

//Also sprach Zarathustra - the full Sunrise intro (the 2001 fanfare),
//down one octave. Three statements of the rising C-G-C fanfare, walking
//front-to-rear across the car. The answering hits land on RR: statement 1
//falls major-to-minor (E-Eb), statement 2 inverts it (Eb-E), statement 3
//rises through E-F into the held G climax. Timpani C-G pounding alternates
//across the rear pair between statements. Everything sits low-register on
//purpose so it survives a slow inverter setpoint ingestion rate; the 392Hz
//climax is the only note near the risky top. Total ~16s - the driver can
//throttle-skip it. Edit freely - this table is the whole song.
static const SongNote song[] = {
    //Statement 1: C G C, answered major falling to minor
    { 131, 700, 0 },                 //C3, FL
    {   0, 120, 0 },
    { 196, 700, 1 },                 //G3, FR
    {   0, 120, 0 },
    { 262, 900, 2 },                 //C4, RL
    {   0,  80, 0 },
    { 330, 300, 3 },                 //E4, RR
    { 311, 500, 3 },                 //Eb4, RR
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
    { 311, 300, 3 },                 //Eb4, RR
    { 330, 500, 3 },                 //E4, RR
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
    { 330, 350, 3 },                 //E4, RR
    { 349, 350, 0 },                 //F4, FL
    { 392, 1400, 1 },                //G4, FR - held
    {   0, 200, 0 },
    //Timpani finale: C G C G C G C G
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
    { 131, 180, 2 }, {  98, 180, 3 },
};
#define MOTORSONG_LENGTH (sizeof(song) / sizeof(song[0]))

static SongState songState = SONG_IDLE;
static ubyte2 noteIndex = 0;
static ubyte4 timestamp_noteStart = 0;
static ubyte4 timestamp_lastToggle = 0;
static sbyte1 toggleSign = 1;
static bool noteSilenced = FALSE;
static sbyte2 savedLimitPositive[4];
static sbyte2 savedLimitNegative[4];

//Pack and send one AMK setpoint frame straight into the CAN1 FIFO,
//mirroring the layout in canOutput_sendDebugMessage1.
static void MotorSong_sendSetpointFrame(CanManager *canMan, _DriveInverter *motor)
{
    IO_CAN_DATA_FRAME frame;
    frame.id = motor->canIdOutgoing;
    frame.id_format = IO_CAN_STD_FRAME;
    frame.length = 8;
    frame.data[0] = 0;
    frame.data[1] = (ubyte1)((motor->AMK_InverterOn_send << 0) | (motor->AMK_DcOn_send << 1) | (motor->AMK_Enable_send << 2) | (motor->AMK_ErrorReset_send << 3)) & 0x0F;
    frame.data[2] = motor->AMK_TorqueRequest_send;
    frame.data[3] = motor->AMK_TorqueRequest_send >> 8;
    frame.data[4] = motor->AMK_TorqueLimitPositive_send;
    frame.data[5] = motor->AMK_TorqueLimitPositive_send >> 8;
    frame.data[6] = motor->AMK_TorqueLimitNegative_send;
    frame.data[7] = motor->AMK_TorqueLimitNegative_send >> 8;
    CanManager_sendImmediate(canMan, CAN1_LOPRI, &frame, 1);
}

static void MotorSong_silenceAll(CanManager *canMan, _Powertrain *powertrain)
{
    for(ubyte1 i = 0; i < 4; ++i){
        powertrain->motor[i]->AMK_TorqueRequest_send = 0;
        powertrain->motor[i]->AMK_TorqueLimitPositive_send = savedLimitPositive[i];
        powertrain->motor[i]->AMK_TorqueLimitNegative_send = savedLimitNegative[i];
        MotorSong_sendSetpointFrame(canMan, powertrain->motor[i]);
    }
}

void MotorSong_start(_Powertrain *powertrain)
{
    for(ubyte1 i = 0; i < 4; ++i){
        savedLimitPositive[i] = powertrain->motor[i]->AMK_TorqueLimitPositive_send;
        savedLimitNegative[i] = powertrain->motor[i]->AMK_TorqueLimitNegative_send;
        powertrain->motor[i]->AMK_TorqueRequest_send = 0;
        //Clamp limits to the song amplitude so nothing can command drive
        //torque while the song owns the setpoints
        powertrain->motor[i]->AMK_TorqueLimitPositive_send = MOTORSONG_TORQUE_AMPLITUDE;
        powertrain->motor[i]->AMK_TorqueLimitNegative_send = -MOTORSONG_TORQUE_AMPLITUDE;
    }
    noteIndex = 0;
    toggleSign = 1;
    noteSilenced = FALSE;
    songState = SONG_PLAYING;
    IO_RTC_StartTime(&timestamp_noteStart);
    IO_RTC_StartTime(&timestamp_lastToggle);
}

void MotorSong_stop(CanManager *canMan, _Powertrain *powertrain)
{
    if(songState != SONG_PLAYING){
        return;
    }
    MotorSong_silenceAll(canMan, powertrain);
    songState = SONG_IDLE;
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

void MotorSong_safetyUpdate(CanManager *canMan, _Powertrain *powertrain, TorqueEncoder *tps, Sensor *HVILTermSense)
{
    if(songState != SONG_PLAYING){
        return;
    }

    //Driver on the throttle = they want the car, not the concert
    if(tps->calibrated == TRUE && tps->travelPercent > MOTORSONG_SKIP_TPS_PERCENT){
        MotorSong_skip(canMan, powertrain);
        return;
    }

    bool abortSong = FALSE;
    if(HVILTermSense->sensorValue == FALSE){
        abortSong = TRUE;
    }
    for(ubyte1 i = 0; i < 4; ++i){
        if(powertrain->motor[i]->AMK_Error_recieve == TRUE ||
           powertrain->motor[i]->AMK_InverterOn_recieve == FALSE ||
           powertrain->motor[i]->AMK_QuitInverterOn_recieve == FALSE)
        {
            abortSong = TRUE;
        }
    }
    if(abortSong == TRUE){
        MotorSong_stop(canMan, powertrain);
    }
}

void MotorSong_fastTask(CanManager *canMan, _Powertrain *powertrain)
{
    if(songState != SONG_PLAYING){
        return;
    }

    const SongNote *note = &song[noteIndex];
    _DriveInverter *motor = powertrain->motor[note->motorIndex];
    ubyte4 noteElapsed_us = IO_RTC_GetTimeUS(timestamp_noteStart);
    ubyte4 noteDuration_us = (ubyte4)note->duration_ms * 1000;

    //Note over - silence its motor and advance
    if(noteElapsed_us >= noteDuration_us){
        if(motor->AMK_TorqueRequest_send != 0){
            motor->AMK_TorqueRequest_send = 0;
            MotorSong_sendSetpointFrame(canMan, motor);
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
        if(noteSilenced == FALSE && motor->AMK_TorqueRequest_send != 0){
            motor->AMK_TorqueRequest_send = 0;
            MotorSong_sendSetpointFrame(canMan, motor);
        }
        noteSilenced = TRUE;
        return;
    }

    //Voice the note: square wave at freq_Hz via alternating torque sign
    ubyte4 halfPeriod_us = 500000UL / note->freq_Hz;
    if(IO_RTC_GetTimeUS(timestamp_lastToggle) >= halfPeriod_us){
        IO_RTC_StartTime(&timestamp_lastToggle);
        toggleSign = -toggleSign;
        motor->AMK_TorqueRequest_send = (sbyte2)(toggleSign * MOTORSONG_TORQUE_AMPLITUDE);
        MotorSong_sendSetpointFrame(canMan, motor);
    }
}
