#include "launchControl.h"
#include "wheelSpeeds.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "drs.h"
#include "PID.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

extern Sensor Sensor_LCButton;
extern Sensor Sensor_RightKnob;

//LC Status Flags
//nibble 1
static const ubyte1 LC_ready = 1;
static const ubyte1 LC_active = 2;
static const ubyte1 LC_initalCurve = 4;
static const ubyte1 LC_unused_1 = 8;
//nibble 2

static const ubyte1 LC_engaged = 0x10;
static const ubyte1 LC_constantSpeedOverride = 0x20;
static const ubyte1 LC_speedMode = 0x40;
static const ubyte1 LC_unused_2 = 0x80;

//Initial Torque Setpoints
typedef enum {
    MODE_ONE = 105,
    MODE_TWO = 110,
    MODE_THREE = 115,
    MODE_FOUR = 120,
    MODE_SAFE_ONE = 40,
    MODE_SAFE_TWO = 50
 } LC_Starting_Torques;

LaunchControl *LaunchControl_new(){
    LaunchControl* lc = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    // malloc returns NULL if it fails to allocate memory
    if (lc == NULL)
        return NULL;

    lc->targetSlip = 0.15;
    lc->estimatedSlip = 0.00;

    lc->pidTorque = PID_new(20, 0, 0, lc->targetSlip, 100, 231, 0, 0, 0);
    lc->torqueRequest = 0;

    lc->pidSpeed= PID_new(20, 0, 0, lc->targetSlip, 100, 6000, 0, 0, 0);
    lc->speedRequest = 0;

    lc->safteyTimer = NULL;
    lc->lcReady = FALSE;
    lc->lcActive = FALSE;
    lc->flags = 0x00;

    /** Variables for constantSpeedTestOverride Function. 
     * Enabling this mode disabled Launch Control & 
     * changes button function to act as a cruise control targeting a specified speed */
    lc->constantSpeedTestOverride = FALSE;
    lc->overrideTestSpeedCommand = 1000; // CONSTANT SPEED TARGET

    lc->buttonDebug = 0; // This exists as a holdover piece of code to what I presume is debugging which button was which on the steering wheel. should remove / place elsewhere
    return lc;
}

void LaunchControl_initialTorqueCurve(LaunchControl* lc, MotorController* mcm){
    lc->torqueRequest = MODE_TWO + (float4)( MCM_getMotorRPM(mcm) * 0.35 ); // Tunable Values will be the inital Torque Request @ 0 and the scalar factor
}

void LaunchControl_estimateSlipRatio(LaunchControl *lc, MotorController *mcm, WheelSpeeds *wss){
    float4 RearR = WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE) + 0.5;
    float4 FrontL = WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5;
    lc->estimatedSlip = ( RearR / FrontL ) -1;
}


void LaunchControl_calculateMCUCommand(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, WheelSpeeds *wss, DRS *drs){
    
    LaunchControl_checkState(lc, tps, bps, mcm, drs);
    if(lc->lcActive)
    {
        LaunchControl_estimateSlipRatio(lc,mcm,wss);
        if( MCM_getGroundSpeedKPH(mcm) < 5 ) {
            LaunchControl_initialTorqueCurve(lc, mcm);
        }
        else {    
            lc->pidTorque->controllerMaxima = (tps->travelPercent * 231);
            float processVariable = lc->estimatedSlip;
            float targetValue = lc->targetSlip;
            float outputVariable = MCM_getCommandedTorque(mcm);
            float torqueRequest = PID_computeOutput(lc->pidTorque, targetValue, processVariable, outputVariable);
            sbyte2 torqueDeciNM = torqueRequest * 10;
            MCM_set_launchControlTorqueRequest(mcm,torqueDeciNM);

            outputVariable = MCM_(mcm);
            float speedRequest = PID_computeOutput(lc->pidTorque, targetValue, processVariable, outputVariable);
            MCM_set_launchControlSpeedRequest(mcm,speedRequest);
        }
    }
}

void LaunchControl_checkState(LaunchControl *lc, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    sbyte2 speedKph         = MCM_getGroundSpeedKPH(mcm);

    /* LC STATUS CONDITIONS *//*
     * lcReady = FALSE && lcActive = FALSE -> NOTHING HAPPENS
     * lcReady = TRUE  && lcActive = FALSE -> We are in the prep stage for lc, and all entry conditions for being in prep stage have and continue to be monitored
     * lcReady = FALSE && lcActive = TRUE  -> We have left the prep stage by releasing the lc button on the steering wheel, stay in Launch until exit conditions are met
     * AT ALL TIMES, EXIT CONDITIONS ARE CHECKED FOR BOTH STATES
    */

    /**
     * Launch Control Pre-Staging Operations:
     * If the car is near 0 kph (in case of wss float issues) & the Launch Button is pressed,
     * we initialise a 0.5 second timer to confirm a valid Launch attempt
     * Once this timer reaches maturity, we are now in "ready" state
     * The driver can now fully press TPS/APPS without moving car
     * 
     * Upon button release, we are now in "active" state and will proceed with our launch as intended -> car go eeeeeeeee (e-motor sounds)
     * 
     * At any time, an exit condition can be triggered to reset this staging operation and cancel our launch attempt
     */
    // Handle the LC Active Entry first so we don't need to displace the lcReady = FALSE trigger to a differnet place (if a button is released, lcReady would otherwise report false before we try to enter lcActive)
    if( lc->lcReady == TRUE && Sensor_LCButton.sensorValue == FALSE ){
        lc->pidTorque->totalError = 170; // Error should be set here, so for every launch we reset our error to this value (check if this is the best value)
        lc->lcActive = TRUE;
        lc->lcReady = FALSE;
    }
    
    if(Sensor_LCButton.sensorValue == TRUE && speedKph < 1 && bps->percent < 0.05 ) {
        if (lc->safteyTimer == 0){
            IO_RTC_StartTime(&lc->safteyTimer);
            // DRS_open(drs); // Visual Indicator of LC staging
        }
        else if (IO_RTC_GetTimeUS(lc->safteyTimer) >= 50000) {
            lc->lcReady = TRUE;
            // DRS_close(drs); // Visual Indicator of LC staging
            lc->safteyTimer = 0; // We don't need to track the time anymore
        }
    } else { lc->lcReady = FALSE; }

    if( tps->tps0_percent < 0.90 || tps->tps0_percent < 0.90 || bps->percent > 0.05 ){
        lc->lcActive = FALSE;
    }
    
    MCM_update_LC_activeStatus(mcm, (bool)lc->lcActive);
    MCM_update_LC_readyStatus(mcm, (bool)lc->lcReady);
}

ubyte1 LaunchControl_getStatus(LaunchControl *lc){ 
    //Ready;
    if (lc->lcReady == TRUE)
    {
        lc->flags |= LC_ready;
    }
    else
    {
        lc->flags &= ~LC_ready;
    }
    //Active;
    if (lc->lcActive == TRUE)
    {
        lc->flags |= LC_active;
    }
    else
    {
        lc->flags &= ~LC_active;
    }
    //Predetermined Torque Curve;
    if (lc->initialCurve == TRUE)
    {
        lc->flags |= LC_initalCurve;
    }
    else
    {
        lc->flags &= ~LC_initalCurve;
    }
    //LC Requesting above MaxTorque;
    if (lc->overTorque == TRUE)
    {
        lc->flags |= LC_unused_1;
    }
    else
    {
        lc->flags &= ~LC_unused_1;
    }
    //LC In Speed Mode;
    if (lc->lcSpeedCommand != 0)
    {
        lc->flags |= LC_engaged;
    }
    else
    {
        lc->flags &= ~LC_engaged;
    }
    //LC Used for Constant Speed Tests;
    if (lc->constantSpeedTestOverride == TRUE)
    {
        lc->flags |= LC_constantSpeedOverride;
    }
    else
    {
        lc->flags &= ~LC_constantSpeedOverride;
    }
    //Reports if undershooting Slip Target;
    if (lc->pidTorque->setpoint >= lc->slipRatioThreeDigits)
    {
        lc->flags |= LC_speedMode;
    }
    else
    {
        lc->flags &= ~LC_speedMode;
    }
    //Reports if overshooting Slip Target;
    if (lc->pidTorque->setpoint <= lc->slipRatioThreeDigits)
    {
        lc->flags |= LC_unused_2;
    }
    else
    {
        lc->flags &= ~LC_unused_2;
    }
    return lc->flags; 
}

ubyte1 LaunchControl_getButtonDebug(LaunchControl *lc) { return lc->buttonDebug; }
/**
void LaunchControl_checkRotary(LaunchControl *lc)
{
    switch(Rotary_getLeftPosition()){
        case ROTARY_POS_1:
            PID_updateSettings(lc->pidTorque, setpoint, 325);
            PID_updateSettings(lc->pidTorque, frequency, 1);
            lc->initialTorque = Standard;
            lc->constA = 7;
            lc->constB = 20;
            break;

        case ROTARY_POS_2:
            PID_updateSettings(lc->pidTorque, setpoint, 325);
            PID_updateSettings(lc->pidTorque, frequency, 1);
            lc->initialTorque = Standard;
            lc->constA = 13;
            lc->constB = 40;
            break;

        case ROTARY_POS_3:
            PID_updateSettings(lc->pidTorque, setpoint, 315);
            PID_updateSettings(lc->pidTorque, frequency, 1);
            lc->initialTorque = Standard;
            lc->constA = 7;
            lc->constB = 20;
            break;

        case ROTARY_POS_4:
            PID_updateSettings(lc->pidTorque, setpoint, 300);
            PID_updateSettings(lc->pidTorque, frequency, 1);
            lc->initialTorque = Medium_High;
            lc->constA = 13;
            lc->constB = 40;
            break;

        case ROTARY_POS_5:
            PID_updateSettings(lc->pidTorque, setpoint, 325);
            PID_updateSettings(lc->pidTorque, frequency, 1);
            lc->initialTorque = High;
            lc->constA = 19;
            lc->constB = 60;
            break;
        
        case ROTARY_POS_5: // Make this a full hard-coded launch?
            PID_updateSettings(lc->pidTorque, setpoint, 325);
            PID_updateSettings(lc->pidTorque, frequency, 1);
            lc->initialTorque = Standard;
            lc->constA = 7;
            lc->constB = 20;
            break;
        
        case ROTARY_POS_6:
            PID_updateSettings(lc->pidTorque, setpoint, 150);
            PID_updateSettings(lc->pidTorque, frequency, 2);
            lc->initialTorque = Rain_Guess_2;
            lc->constA = 3;
            lc->constB = 10;
            break;
    }
}
*/