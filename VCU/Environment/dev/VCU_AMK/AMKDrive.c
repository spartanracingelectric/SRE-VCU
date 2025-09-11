/*****************************************************************************
 * AMKDrive.c - Drive Inverter (DI)
 * Initial Author: Shinika Balasundar
 ******************************************************************************
 * Calculated initial Torque to AMKs, Sends values to AMKs, and parses messages from AMKs
 ****************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_Driver.h"
#include "IO_CAN.h"

#include "AMKDrive.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "sensorCalculations.h"
#include "readyToDriveSound.h"
#include "daqSensors.h"

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_HVILTerminationSense;

// Global variables for state management
ubyte4 timestamp_Precharge = 0;
bool prevHVILState = FALSE;

// Torque calculation constants
#define NOMINAL_TORQUE_NM 9.8
#define MAX_TORQUE_NM 21.0f
#define TORQUE_SCALING_FACTOR 10000
#define TORQUE_LIMIT_SCALE_FACTOR 10

// Timing constants
#define PRECHARGE_SOAK_TIME_US 10000000UL // 10 seconds
#define RTDS_DURATION_US 1500000UL // 1.5 seconds ready to drive sound duration
#define RTDS_CHANNEL 1

// Masking bits for CAN parsing
#define SYSTEM_READY_STATUS 0x01
#define ERROR_STATUS 0x02
#define WARNING_STATUS 0x04
#define QUIT_DC_ON_STATUS 0x08
#define DC_ON_VAL_STATUS 0x10
#define QUIT_INVERTER_ON_STATUS 0x20
#define INVERTER_ON_STATUS 0x40
#define DERATING_STATUS 0x80

// Calculated constants
#define TORQUE_MAX_CALC ((MAX_TORQUE_NM / NOMINAL_TORQUE_NM) / 100.0f)
#define TORQUE_LIMIT_POS_01NM ((ubyte2)(MAX_TORQUE_NM * TORQUE_LIMIT_SCALE_FACTOR)) // 21Nm in 0.1 Nm units
#define TORQUE_LIMIT_NEG_01NM 0 // set -210 for regen

// MAIN FUNCTIONS
// Creates a new DriveInverter object
_DriveInverter* AmkDriver_new(DI_Location_Address location_address)
{
    _DriveInverter* me = (_DriveInverter*)malloc(sizeof(_DriveInverter));
 
        me->location_address = location_address;

        me->canIdOutgoing = DI_BASE_CAN_ID_OUTGOING + me->location_address;
        me->canIdIncoming = DI_BASE_CAN_ID_INCOMING + me->location_address;

        int startUpStage = 0;

        // Setpoints (commands, outgoing: 0x183 + offset)
        me->AMK_bInverterOn = FALSE;
        me->AMK_bDcOn = FALSE;
        me->AMK_bEnable = FALSE;
        me->AMK_bErrorReset = FALSE;
        me->AMK_TorqueSetpoint = 0;
        me->AMK_TorqueLimitPositiv = 0;
        me->AMK_TorqueLimitNegativ = 0;

        // Actual Values 1 (incoming: 0x282 + offset)
        me->AMK_bSystemReady = FALSE;
        me->AMK_bError = FALSE;
        me->AMK_bWarn = FALSE;
        me->AMK_bQuitDcOn = FALSE;
        me->AMK_bDcOnVal = FALSE;
        me->AMK_bQuitInverterOnVal = FALSE;
        me->AMK_bInverterOnVal = FALSE;
        me->AMK_bDerating = FALSE;
        me->AMK_ActualVelocity = 0.0;
        me->AMK_TorqueCurrent = 0.0;
        me->AMK_MagnetizingCurrent = 0.0;

        // Actual Values 2 (incoming: 0x284 + offset)
        me->AMK_TempMotor = 0.0; // 0.1degC
        me->AMK_TempInverter = 0.0;
        me->AMK_ErrorInfo = 0.0;
        me->AMK_TorqueFeedback = 0.0; // % 0.1 Nm

    return me;
}

enum InverterStatus {
    RELAY_OFF = 0,
    RELAY_ON_SENDING_CAN = 1,
    PRECHARGE_DC_ENABLE = 2,
    DRIVER_ENABLE = 3,
    READY_TO_DRIVE_INVERTER_ON = 4,
    TORQUE_LIMIT_SET = 5,
    TORQUE_SETPOINTS_ACTIVE = 6
};


// Calculates torque commands based on sensor inputs
// SRE-7 Update: CAN has a scaling of 0.1. So send 214% of 9.8nm (nominal) to get 21nm. So 0.25 is 25% of 9.8 and 2.14 is 214% of 9.8.
void DI_calculateCommands(_DriveInverter* me, TorqueEncoder *tps, BrakePressureSensor *bps){
    sbyte2 torqueOutput = 0;
    sbyte2 appsTorque = 0;
    sbyte2 bpsTorque = 0;

    float4 appsOutputPercent;

    /*
    TorqueEncoder_getOutputPercent(tps, &appsOutputPercent);

    sbyte2 torqueMax = (me->AMK_TorqueLimitPositiv / 100); // this should give 21 or the max value 
    torqueMax = torqueMax / 9.8; //Needs to be divided by the nominal torque and give 2.14 as max since 2.14 is 214% of 9.8. Should scale to any max value

    appsTorque = torqueMax * getPercent(appsOutputPercent, 0, 1, TRUE) - 0 * getPercent(appsOutputPercent, 0, 0, TRUE);
    bpsTorque = 0 - (0 - 0) * getPercent(bps->percent, 0, 0, TRUE);
    */ 

    //Vehicle Testing works on Test-Bench
    TorqueEncoder_getIndividualSensorPercent(tps, 0, &appsOutputPercent);
    appsOutputPercent = appsOutputPercent * TORQUE_SCALING_FACTOR; // Scale to 0-100%

    float4 torqueMax = TORQUE_MAX_CALC;
    //float4 torqueMax = (((float4)me->AMK_TorqueSetpoint / 10.0 / 9.8) / 100.0);

    torqueOutput = appsOutputPercent * torqueMax; 

    DI_commandTorque(me, torqueOutput);
    DI_getCommandedTorque(me);

}

// Calculates inverter control state machine
void DI_calculateInverterControl(_DriveInverter* me, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds, _DAQSensors *d1){
    const bool hvil_ok = (HVILTermSense && HVILTermSense->sensorValue == TRUE);

     switch (me->startUpStage){ 
        case RELAY_OFF:
            DI_handleRelayOff(me);
            break;

        case RELAY_ON_SENDING_CAN:
            DI_handleRelayOnSendingCAN(me);
            break;

        case PRECHARGE_DC_ENABLE:
            DI_handlePrechargeDCEnable(me, hvil_ok, d1);
            break;

        case DRIVER_ENABLE: 
            DI_handleDriverEnable(me, tps);
            break;

        case READY_TO_DRIVE_INVERTER_ON:
            DI_handleReadyToDriveInverterOn(me, rtds);
            break;

        case TORQUE_LIMIT_SET: 
            DI_handleTorqueLimitSet(me);
            break;

        case TORQUE_SETPOINTS_ACTIVE:
            DI_handleTorqueSetpointsActive(me, hvil_ok);
            break;

        default:
        //We lost track of the sequence
        me->startUpStage = RELAY_OFF;
        break;
     }
}  

void DI_parseCanMessage(_DriveInverter* me, IO_CAN_DATA_FRAME* diCanMessage){
    int address1 = me->canIdIncoming;
    int address2 = me->canIdIncoming + 2;

    if(diCanMessage->id == address1) {
        DI_parseStatusBits(me, diCanMessage->data[1]);
        DI_parseActualValues1(me, diCanMessage->data);
    } else if(diCanMessage->id == address2) {
        DI_parseActualValues2(me, diCanMessage->data);
    }
}

void DI_commandTorque(_DriveInverter* me, sbyte2 newTorque){
     me->AMK_TorqueSetpoint = newTorque;
}

sbyte2 DI_getCommandedTorque(_DriveInverter* me){
     return me->AMK_TorqueSetpoint;
}

// Sets inverter command parameters
static inline void DI_cmd(_DriveInverter* me, bool invOn, bool dcOn, bool enable, bool errReset, sbyte2 torque01Nm, ubyte2 limPos01Nm, ubyte2 limNeg01Nm){
    me->AMK_bInverterOn = invOn;
    me->AMK_bDcOn = dcOn;
    me->AMK_bEnable = enable;
    me->AMK_bErrorReset = errReset;
    me->AMK_TorqueSetpoint = torque01Nm;
    me->AMK_TorqueLimitPositiv = limPos01Nm;
    me->AMK_TorqueLimitNegativ = limNeg01Nm;

}


static void DI_handleRelayOff(_DriveInverter* me) {
    if(Sensor_RTDButton.sensorValue == FALSE){
        IO_DO_Set(IO_DO_00, TRUE);
        me->startUpStage = RELAY_ON_SENDING_CAN;
    } else {
        IO_DO_Set(IO_DO_00, FALSE);
        me->startUpStage = RELAY_OFF;
    }
}

static void DI_handleRelayOnSendingCAN(_DriveInverter* me) {
    //MCM relay on, we can now start sending safe CAN messages
    DI_cmd(me, FALSE, FALSE, FALSE, FALSE, 0, 0, 0);

    if(me->AMK_bSystemReady == TRUE && me->AMK_bError == FALSE){ 
        timestamp_Precharge = 0;
        me->startUpStage = PRECHARGE_DC_ENABLE;
    }
}

static void DI_handlePrechargeDCEnable(_DriveInverter* me, bool hvil_ok, _DAQSensors *d1) {
    //Precharge needs to have occured to now send the new message 
    if (hvil_ok && timestamp_Precharge == 0){
        IO_RTC_StartTime(&timestamp_Precharge);
    } 
    if(hvil_ok && IO_RTC_GetTimeUS(timestamp_Precharge) >= PRECHARGE_SOAK_TIME_US && d1->DetectionPCB == 1) // After 10 Seconds // Added Integration for DetectionPCB
    { 
        DI_cmd(me, FALSE, TRUE, FALSE, FALSE, 0, 0, 0);
    }  
    if(me->AMK_bDcOnVal == TRUE && me->AMK_bQuitDcOn == TRUE){
        me->startUpStage = DRIVER_ENABLE;
        //Main contactor close code here (look at old MCM relay logic)
        //Open precharge relay code here (look at old MCM relay logic)
    }
}

static void DI_handleDriverEnable(_DriveInverter* me, TorqueEncoder *tps) {
    if(Sensor_RTDButton.sensorValue == TRUE && tps->calibrated == TRUE /*Uncomment in the future: && tps->travelPercent < .05*/){
        DI_cmd(me, FALSE, TRUE, TRUE, FALSE, 0, 0, 0);
        me->startUpStage = READY_TO_DRIVE_INVERTER_ON; 
    }
}

static void DI_handleReadyToDriveInverterOn(_DriveInverter* me, ReadyToDriveSound *rtds) {
    if(Sensor_RTDButton.sensorValue == FALSE && me->AMK_bEnable == TRUE /*Add in brakes being pressed*/){
        DI_cmd(me, TRUE, TRUE, TRUE, FALSE, 0, 0, 0);
    }
    if(me->AMK_bInverterOnVal == TRUE && me->AMK_bQuitInverterOnVal == TRUE){
        RTDS_setVolume(rtds, RTDS_CHANNEL, RTDS_DURATION_US);
        me->startUpStage = TORQUE_LIMIT_SET;
    } 
}

static void DI_handleTorqueLimitSet(_DriveInverter* me) {
    DI_cmd(me, TRUE, TRUE, TRUE, FALSE, 0, TORQUE_LIMIT_POS_01NM, TORQUE_LIMIT_NEG_01NM); // 25Nm for positive torque limit > Will need to find a way to make this global for the future (make sure correct on CAN)
    if(me->AMK_bError == FALSE){
        me->startUpStage = TORQUE_SETPOINTS_ACTIVE;
    }
}

static void DI_handleTorqueSetpointsActive(_DriveInverter* me, bool hvil_ok) {
    // SRE-7 Update: 21Nm -> Will need to find a way to make this global for the future (make sure correct on CAN)
    //SRE-7 Update: Make -21 for Regen
    DI_cmd(me, TRUE, TRUE, TRUE, FALSE, 0, TORQUE_LIMIT_POS_01NM, TORQUE_LIMIT_NEG_01NM); 
    if(me->AMK_bError == TRUE || !hvil_ok){
        me->startUpStage = RELAY_ON_SENDING_CAN; 
    }
}

static void DI_parseStatusBits(_DriveInverter* me, ubyte1 data) {
    // System ready status
    me->AMK_bSystemReady = ((data & SYSTEM_READY_STATUS) > 0);
    // Error status
    me->AMK_bError = ((data & ERROR_STATUS) > 0);
    // Warnings status
    me->AMK_bWarn = ((data & WARNING_STATUS) > 0);
    // Quit DC on status
    me->AMK_bQuitDcOn = ((data & QUIT_DC_ON_STATUS) > 0);
    // DC on status
    me->AMK_bDcOnVal = ((data & DC_ON_VAL_STATUS) > 0);
    // Quit inverter on status
    me->AMK_bQuitInverterOnVal = ((data & QUIT_INVERTER_ON_STATUS) > 0);
    // Inverter on status
    me->AMK_bInverterOnVal = ((data & INVERTER_ON_STATUS) > 0);
    // Derating value
    me->AMK_bDerating = ((data & DERATING_STATUS) > 0);
}

static void DI_parseActualValues1(_DriveInverter* me, ubyte1* data) {
    // Speed value
    me->AMK_ActualVelocity = (data[3] << 8 | data[2]);
    // Torque current
    me->AMK_TorqueCurrent = (data[5] << 8 | data[4]);
    // Magnetized current
    me->AMK_MagnetizingCurrent = (data[7] << 8 | data[6]);
}

static void DI_parseActualValues2(_DriveInverter* me, ubyte1* data) {
    // Motor temperature
    me->AMK_TempMotor = (float4)(data[1] << 8 | data[0]);
    // Inverter temperature
    me->AMK_TempInverter = (float4)(data[3] << 8 | data[2]);
    // Diagnostic number
    me->AMK_ErrorInfo = (ubyte2)(data[5] << 8 | data[4]);
    // Torque feedback
    me->AMK_TorqueFeedback = (float4)(data[7] << 8 | data[6]);
}

