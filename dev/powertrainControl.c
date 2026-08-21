/*****************************************************************************
 * powertrainControl.c 
 * Formerly (AMKDrive.h - Drive Inverter (DI))
 * Initial Author: Shinika Balasundar
 * Additional Author: Shaun Gilmore, Akash Karthik
 ******************************************************************************
 * Calculated initial Torque to AMKs, Sends values to AMKs, and parses messages from AMKs
 ****************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_Driver.h"
#include "IO_CAN.h"

#include "powertrainControl.h"
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

_DriveInverter* AmkDriver_new(DI_Location_Address location_address)
{
    _DriveInverter* me = (_DriveInverter*)malloc(sizeof(_DriveInverter));
 
        me->location_address = location_address;

        me->canIdOutgoing = DI_BASE_CAN_ID_OUTGOING + me->location_address;
        me->canIdIncoming = DI_BASE_CAN_ID_INCOMING + me->location_address;

        me->startUpStage = 0;

        // Setpoints (commands, outgoing: 0x183 + offset)
        me->AMK_InverterOn_send = FALSE;
        me->AMK_DcOn_send = FALSE;
        me->AMK_Enable_send = FALSE;
        me->AMK_ErrorReset_send = FALSE;
        me->AMK_TorqueRequest_send = 0;
        me->AMK_TorqueLimitPositive_send = 0;
        me->AMK_TorqueLimitNegative_send = 0;

        // Actual Values 1 (incoming: 0x282 + offset)
        me->AMK_SystemReady_recieve = FALSE;
        me->AMK_Error_recieve = FALSE;
        me->AMK_Warn_recieve = FALSE;
        me->AMK_QuitDcOn_recieve = FALSE;
        me->AMK_DcOn_recieve = FALSE;
        me->AMK_QuitInverterOn_recieve = FALSE;
        me->AMK_InverterOn_recieve = FALSE;
        me->AMK_Derating_recieve = FALSE;
        me->AMK_ActualVelocity_recieve = 0.0;
        me->AMK_TorqueCurrentRaw_recieve = 0.0;
        me->AMK_MagnetizingCurrentRaw_recieve = 0.0;

        // Actual Values 2 (incoming: 0x284 + offset)
        me->AMK_TempMotor_recieve = 0.0; // 0.1degC
        me->AMK_TempInverter_recieve = 0.0;
        me->AMK_ErrorInfo_recieve = 0.0;
        me->AMK_TorqueFeedback_recieve = 0.0; // % 0.1 Nm

        me->AMK_ID110 = 107200;
    return me;
}

enum InverterStatus {
    RELAY_OFF = 0,
    RELAY_ON_SENDING_CAN = 1,
    PRECHARGE_DC_ENABLE = 2,
    DRIVER_ENABLE = 3,
    READY_TO_DRIVE_INVERTER_ON = 4,
    TORQUE_LIMIT_SET = 5,
    TORQUE_REQUEST_ACTIVE = 6
};



void DI_calculateInverterControl(_Powertrain *powertrain, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds, _DAQSensors *d1){
    for(ubyte1 i = 0; i < 4; ++i){
        if(powertrain->motor[i]->AMK_Error_recieve == TRUE){
            powertrain->motor[i]->startUpStage = RELAY_OFF;
            powertrain->motor[i]->AMK_InverterOn_send = FALSE;
            powertrain->motor[i]->AMK_DcOn_send = FALSE;
            powertrain->motor[i]->AMK_Enable_send = FALSE;
            powertrain->motor[i]->AMK_ErrorReset_send = TRUE;
            powertrain->motor[i]->AMK_TorqueRequest_send = 0;
            powertrain->motor[i]->AMK_TorqueLimitPositive_send = 0;
            powertrain->motor[i]->AMK_TorqueLimitNegative_send = 0; 
            powertrain->rtdsPlayed = FALSE;
        }
        switch (powertrain->motor[i]->startUpStage){ 
            case RELAY_OFF:
                IO_DO_Set(IO_DO_00, TRUE);
                powertrain->motor[i]->startUpStage = RELAY_ON_SENDING_CAN;
            break;
            //MCM relay on, we can now start sending safe CAN powertrain->motor[i]ssages
            case RELAY_ON_SENDING_CAN:
                powertrain->motor[i]->AMK_InverterOn_send = FALSE;
                powertrain->motor[i]->AMK_DcOn_send = FALSE;
                powertrain->motor[i]->AMK_Enable_send = FALSE;
                powertrain->motor[i]->AMK_ErrorReset_send = FALSE;
                powertrain->motor[i]->AMK_TorqueRequest_send = 0;
                powertrain->motor[i]->AMK_TorqueLimitPositive_send = 0;
                powertrain->motor[i]->AMK_TorqueLimitNegative_send = 0; 
                if(powertrain->motor[i]->AMK_SystemReady_recieve == TRUE && powertrain->motor[i]->AMK_Error_recieve == FALSE){ 
                    powertrain->motor[i]->startUpStage = PRECHARGE_DC_ENABLE;
                }
            break;
            //Precharge needs to have occured to now send the new powertrain->motor[i]ssage 
            case PRECHARGE_DC_ENABLE:
                if(HVILTermSense->sensorValue == TRUE){ // TODO: GET THE PRECHARGE BOARD DETECTION HERE
                    powertrain->motor[i]->AMK_DcOn_send = TRUE;
                }  

                if(powertrain->motor[i]->AMK_DcOn_recieve == TRUE && powertrain->motor[i]->AMK_QuitDcOn_recieve == TRUE){
                    powertrain->motor[i]->startUpStage = DRIVER_ENABLE;
                    //Main contactor close code here (look at old MCM relay logic)
                    //Open precharge relay code here (look at old MCM relay logic)
                }
            break;
            case DRIVER_ENABLE: 
                if(Sensor_RTDButton.sensorValue == TRUE && 
                    tps->calibrated == TRUE && 
                    bps->calibrated == TRUE &&
                    bps->brakesAreOn == TRUE &&
                    tps->travelPercent < 0.05)
                {
                    powertrain->motor[i]->AMK_Enable_send = TRUE;
                    powertrain->motor[i]->startUpStage = READY_TO_DRIVE_INVERTER_ON;
                }
            break;
            case READY_TO_DRIVE_INVERTER_ON:
                if(powertrain->motor[i]->AMK_Enable_send == TRUE){ 
                    powertrain->motor[i]->AMK_InverterOn_send = TRUE;
                }
                if(powertrain->motor[i]->AMK_InverterOn_recieve == TRUE && powertrain->motor[i]->AMK_QuitInverterOn_recieve == TRUE){
                    powertrain->motor[i]->startUpStage = TORQUE_LIMIT_SET;
                } 
            break;
            case TORQUE_LIMIT_SET:
                powertrain->motor[i]->AMK_TorqueLimitPositive_send = 210; // should be max tq 25Nm -> Will need to find a way to make this global for the future (make sure correct on CAN)
                powertrain->motor[i]->AMK_TorqueLimitNegative_send = 0; // some constant for regen
                if(powertrain->motor[i]->AMK_Error_recieve == FALSE){
                    powertrain->motor[i]->startUpStage = TORQUE_REQUEST_ACTIVE;
                }
            break;
            case TORQUE_REQUEST_ACTIVE:
                if(powertrain->motor[i]->AMK_Error_recieve == TRUE || HVILTermSense->sensorValue == FALSE){
                    powertrain->rtdsPlayed = FALSE;
                    powertrain->motor[i]->startUpStage = RELAY_ON_SENDING_CAN; 
                }
            break;

            default:
            //We lost track of the sequence
            powertrain->motor[i]->startUpStage = RELAY_OFF; //lv off (should never happen big oh no)
            break;
        }
    }
    bool allInvertersOn = TRUE;

    for(ubyte1 i = 0; i < 4; ++i)
    {
        if(powertrain->motor[i]->AMK_InverterOn_recieve == FALSE ||
        powertrain->motor[i]->AMK_QuitInverterOn_recieve == FALSE)
        {
            allInvertersOn = FALSE;
            break;
        }
    }

    if(allInvertersOn == TRUE && powertrain->rtdsPlayed == FALSE)
    {
        RTDS_setVolume(rtds, 1, 1500000);
        powertrain->rtdsPlayed = TRUE;
    }
}  

void DI_parseCanMessage(_DriveInverter* me, IO_CAN_DATA_FRAME* diCanMessage){

    //safety.c ubyte4 flags merge together -> enhancement

    int address1 = me->canIdIncoming;
    int address2 = me->canIdIncoming + 2;

    ubyte1 systemReadyBitMask = 1;
    ubyte1 errorBitMask = 2;
    ubyte1 warningBitMask = 4;
    ubyte1 quitDcOnBitMask = 8;
    ubyte1 quitDcOnValBitMask = 0x10;
    ubyte1 quitInverterOnBitMask = 0x20;
    ubyte1 inverterOnBitMask = 0x40;
    ubyte1 deratingBitMask = 0x80;

    if(diCanMessage->id == address1) {
        // System ready status
        me->AMK_SystemReady_recieve = ((diCanMessage->data[1] & systemReadyBitMask) > 0);
        // Error status
        me->AMK_Error_recieve = ((diCanMessage->data[1] & errorBitMask) > 0);
        // Warnings status
        me->AMK_Warn_recieve = ((diCanMessage->data[1] & warningBitMask) > 0);
        // Quit DC on status
        me->AMK_QuitDcOn_recieve = ((diCanMessage->data[1] & quitDcOnBitMask) > 0);
        // DC on status
        me->AMK_DcOn_recieve = ((diCanMessage->data[1] & quitDcOnValBitMask) > 0);
        // Quit inverter on status
        me->AMK_QuitInverterOn_recieve = ((diCanMessage->data[1] & quitInverterOnBitMask) > 0);
        // Inverter on status
        me->AMK_InverterOn_recieve = ((diCanMessage->data[1] & inverterOnBitMask) > 0);
        // Derating value
        me->AMK_Derating_recieve = ((diCanMessage->data[1] & deratingBitMask) > 0);
        // Speed value
        me->AMK_ActualVelocity_recieve = (diCanMessage->data[3] << 8 | diCanMessage->data[2]);
        // Torque current raw value
        me->AMK_TorqueCurrentRaw_recieve = (diCanMessage->data[5] << 8 | diCanMessage->data[4]);
        // Scaled Torque current (actual value)
        me->AMK_TorqueCurrent_recieve = me->AMK_TorqueCurrentRaw_recieve * me->AMK_ID110 / 16384.0;
        // Magnetized current
        me->AMK_MagnetizingCurrentRaw_recieve = (diCanMessage->data[7] << 8 | diCanMessage->data[6]);
        // Scaled Magnetized current (actual value)
        me->AMK_MagnetizingCurrent_recieve = me->AMK_MagnetizingCurrentRaw_recieve * me->AMK_ID110 / 16384.0;

    } else if(diCanMessage->id == address2) {
         // Motor temperature
        me->AMK_TempMotor_recieve = (float4)(diCanMessage->data[1] << 8 | diCanMessage->data[0]);
        // Inverter temperature
        me->AMK_TempInverter_recieve = (float4)(diCanMessage->data[3] << 8 | diCanMessage->data[2]);
        // Diagnostic number
        me->AMK_ErrorInfo_recieve = (ubyte2)(diCanMessage->data[5] << 8 | diCanMessage->data[4]);
        // Torque feedback
        me->AMK_TorqueFeedback_recieve = (float4)(diCanMessage->data[7] << 8 | diCanMessage->data[6]);
    }
}

_Powertrain* Powertrain_new(){
    _Powertrain* me = (_Powertrain*)malloc(sizeof(_Powertrain));
        me->powertrainMode = AWD;
        me->rtdsPlayed = FALSE;
        // Code Convention: Motors stored in following order - [FL,FR,RL,RR]
        for(ubyte1 i = 0; i < 4; ++i)
        {
            me->motor[i] = (_DriveInverter*)malloc(sizeof(_DriveInverter));
        }
        me->motor[0] = AmkDriver_new(FRONT_LEFT);
        me->motor[1] = AmkDriver_new(FRONT_RIGHT);
        me->motor[2] = AmkDriver_new(REAR_LEFT);
        me->motor[3] = AmkDriver_new(REAR_RIGHT);

        me->wheelTorque_Nm = 0;
        me->gearRatio_Front = 0;
        me->gearRatio_Rear = 0;
        me->tireDiameter_in = 16;
        me->motorTorque_Nm = 0;

    return me;
}

void Powertrain_controlVehicle(_Powertrain* me, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds, _DAQSensors *d1){
    DI_calculateInverterControl(me, &Sensor_HVILTerminationSense, tps, bps, rtds, d1);
    if(me->powertrainMode != TorqueVectoring){
        Powertrain_calculateTorqueCommands(me, tps, bps);
    }
    else if(me->powertrainMode == TorqueVectoring){
        Powertrain_TorqueVectoring(me, tps, bps, d1);
    }
}

void Powertrain_calculateTorqueCommands(_Powertrain* me, TorqueEncoder *tps, BrakePressureSensor *bps){
    //all four inverters have to be RTD before any torque is allowed
    for(ubyte1 i = 0; i < 4; ++i)
    {
        if(me->motor[i]->startUpStage != TORQUE_REQUEST_ACTIVE)
        {
            for(ubyte1 j = 0; j < 4; ++j)
            {
                me->motor[j]->AMK_TorqueRequest_send = 0;
            }
            return;
        }
    }
    for(ubyte1 i = 0; i < 4; ++i){
        switch (me->powertrainMode){
            case DISABLED:
                me->motor[i]->AMK_TorqueRequest_send = 0;
                break;
            case FWD:
                if(i < 2){
                    me->motor[i]->AMK_TorqueRequest_send = tps->tps0_percent * me->motor[i]->AMK_TorqueLimitPositive_send; //Commanded Torque based on TPS %
                }
                else{
                    me->motor[i]->AMK_TorqueRequest_send = 0;
                }
                break;
            case RWD:
                if(i >= 2){
                    me->motor[i]->AMK_TorqueRequest_send = tps->tps0_percent * me->motor[i]->AMK_TorqueLimitPositive_send; //Commanded Torque based on TPS %
                }
                else{
                    me->motor[i]->AMK_TorqueRequest_send = 0;
                }
                break;
            case AWD:
                me->motor[i]->AMK_TorqueRequest_send = tps->tps0_percent * me->motor[i]->AMK_TorqueLimitPositive_send; //Commanded Torque based on TPS %
                break;
            case TorqueVectoring:
                //Controls stuff & fuinction calls    
                me->motor[i]->AMK_TorqueRequest_send = tps->tps0_percent * me->motor[i]->AMK_TorqueLimitPositive_send; //Commanded Torque based on TPS %
                break;
            case Drift: // Disabling mode for now
                me->motor[i]->AMK_TorqueRequest_send = 0;
                break;

            default:
                me->motor[i]->AMK_TorqueRequest_send = 0; // no torque if we fail
                break;
        }
    }
}

void Powertrain_TorqueVectoring(_Powertrain *me, TorqueEncoder *tps, BrakePressureSensor *bps, _DAQSensors *d1){
    // TV goes here
}