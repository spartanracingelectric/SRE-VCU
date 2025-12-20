/*****************************************************************************
 * powertrainControl.h 
 * Formerly (AMKDrive.h - Drive Inverter (DI))
 * Initial Author: Shinika Balasundar
 * Additional Author: Shaun Gilmore
 ******************************************************************************
 * Calculated initial Torque to AMKs, Sends values to AMKs, and parses messages from AMKs
 ****************************************************************************/

#ifndef _POWERTRAINCONTROL_H
#define _POWERTRAINCONTROL_H

/***** Standard includes *****/
#include <stdlib.h> //Needed for malloc
#include "IO_CAN.h"

#include "IO_Driver.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"
#include "daqSensors.h"



// Base CAN message ID for outgoing "send" message to inverter
#define DI_BASE_CAN_ID_OUTGOING 0x183
// Base CAN message ID for both incoming "recieve" messages  from inverter
#define DI_BASE_CAN_ID_INCOMING 0x282
// Note that the address for "recieve2" is calculated as recieve ID + 2 (plus offset)
#define DI_BASE_CAN_ID_INCOMING2 (DI_BASE_CAN_ID_INCOMING + 2)

//----------------------------------------------------------------------------
// DI_Location_Address - This maps physical location to CAN ID offset (AMK calls this "node address").
// AMK does not specify which inverter is at which side/axle of the car.
// Note: Motor direction is set in Aipex tool, ID32773 'Service bits' bit 16 = 1.
//----------------------------------------------------------------------------
// Offset  Position     Setpoint Actual1 Actual2
// ----------------------------------------
//         Base address  0x183    0x282   0x284
// ----------------------------------------
//    1    Front left    0x184    0x283   0x285
//    2    Front right   0x185    0x284   0x286
//   3-4   NOT USABLE
//    5    Rear left     0x188    0x287   0x289
//    6    Rear right    0x189    0x288   0x290
//----------------------------------------------------------------------------

typedef enum _DI_Location_Address {
    FRONT_LEFT = 1,
    FRONT_RIGHT = 2,
    REAR_LEFT = 5,
    REAR_RIGHT = 6
} DI_Location_Address;

typedef struct _DriveInverter {
    DI_Location_Address location_address;  // CAN ID offset and physical location
    ubyte2 canIdOutgoing;   // Calculated CAN ID for "setpoint" message
    ubyte2 canIdIncoming;   // Calculated CAN ID for "Actual1" message, also used for "Actual2" message

    ubyte1 startUpStage;

    // Setpoints (commands, outgoing: 0x183 + offset)
    bool AMK_InverterOn_send;
    bool AMK_DcOn_send;
    bool AMK_Enable_send;
    bool AMK_ErrorReset_send;
    sbyte2 AMK_TorqueRequest_send;
    sbyte2 AMK_TorqueLimitPositive_send;
    sbyte2 AMK_TorqueLimitNegative_send;

    // Actual Values 1 (incoming: 0x282 + offset)
    bool AMK_SystemReady_recieve;
    bool AMK_Error_recieve;
    bool AMK_Warn_recieve;
    bool AMK_QuitDcOn_recieve;
    bool AMK_DcOn_recieve;
    bool AMK_QuitInverterOn_recieve;
    bool AMK_InverterOn_recieve;
    bool AMK_Derating_recieve;
    float4 AMK_ActualVelocity_recieve;  // RPM (SRE-7 Update: May Need Multiplier)
    float4 AMK_TorqueCurrentRaw_recieve;
    float4 AMK_MagnetizingCurrentRaw_recieve;

    float4 AMK_TorqueCurrent_recieve;
    float4 AMK_MagnetizingCurrent_recieve;
    // Actual Values 2 (incoming: 0x284 + offset)
    float4 AMK_TempMotor_recieve;       // 0.1degC
    float4 AMK_TempInverter_recieve;    // 0.1degC
    ubyte2 AMK_ErrorInfo_recieve;       
    float4 AMK_TorqueFeedback_recieve;        // Nm
    // Read Page 81 of the datasheet to understand why this is important. https://drive.google.com/file/d/1NLSmcrIAneiVMK4Zg_w86_FE9XNZCwvJ/view?usp=drive_link
    float4 AMK_ID110;
    ubyte2 AMK_TorqueMultiplier;

} _DriveInverter;

typedef enum _PowertrainMode {
    DISABLED = 0,
    FWD = 1,
    RWD = 2,
    AWD = 3,
    TorqueVectoring = 4,
    //Novelty Modes
    Drift = 5
} PowertrainMode;

typedef struct _Powertrain {
    PowertrainMode powertrainMode;
    // Code Convention: Motors stored in following order - [FL,FR,RL,RR]
    _DriveInverter* motor[4];
 
    /*
    Due to potential gear ratio (GR) modifications, 
    vehicle control algorithms should focus on maximizing wheel torque, 
    then working backwards to calculate motor torque.

    This should ensure GR changes are accounted for when changing car setups.
    */
    ubyte2 wheelTorque_Nm;
    float4 gearRatio_Front;
    float4 gearRatio_Rear;
    ubyte1 tireDiameter_in;
    ubyte2 motorTorque_Nm;


} _Powertrain;

_DriveInverter* DriveInverter_new(DI_Location_Address location_address);

void DI_calculateInverterControl(_DriveInverter* Idv, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds, _DAQSensors *d1);

void DI_calculateCommands(_DriveInverter* Idv, TorqueEncoder *tps, BrakePressureSensor *bps);

void DI_parseCanMessage(_DriveInverter* Idv, IO_CAN_DATA_FRAME* diCanMessage);

void DI_commandTorque(_DriveInverter* Idv, sbyte2 newTorque);


_Powertrain* Powertrain_new();

void Powertrain_controlVehicle(_Powertrain* me, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds, _DAQSensors *d1);

#endif