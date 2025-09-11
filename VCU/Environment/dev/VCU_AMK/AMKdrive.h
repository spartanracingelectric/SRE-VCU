/*****************************************************************************
 * AMKDrive.h - Drive Inverter (DI)
 * Initial Author: Shinika Balasundar
 ******************************************************************************
 * Calculated initial Torque to AMKs, Sends values to AMKs, and parses messages from AMKs
 ****************************************************************************/

#ifndef _AMKDRIVE_H
#define _AMKDRIVE_H

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

// Base CAN message ID for "setpoint" message - outgoing to inverter
#define DI_BASE_CAN_ID_OUTGOING 0x183
// Base CAN message ID for both "actual" messages - incoming from inverter
// Note that the address for "Actual2" is calculated as this ID + 2 (plus offset)
#define DI_BASE_CAN_ID_INCOMING 0x282

// Torque calculation constants
#define NOMINAL_TORQUE_NM 9.8f
#define MAX_TORQUE_NM 21.0f
#define TORQUE_SCALING_FACTOR 10000
#define TORQUE_LIMIT_SCALE_FACTOR 10

// Timing constants
#define PRECHARGE_SOAK_TIME_US 10000000UL // 10 seconds
#define RTDS_DURATION_US 1500000UL // 1.5 seconds ready to drive sound duration
#define RTDS_CHANNEL 1

// Bit masks for CAN message parsing
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

typedef enum _InverterStatus {
    RELAY_OFF = 0,
    RELAY_ON_SENDING_CAN = 1,
    PRECHARGE_DC_ENABLE = 2,
    DRIVER_ENABLE = 3,
    READY_TO_DRIVE_INVERTER_ON = 4,
    TORQUE_LIMIT_SET = 5,
    TORQUE_SETPOINTS_ACTIVE = 6
} InverterStatus;

typedef struct _DriveInverter {
    DI_Location_Address location_address;  // CAN ID offset and physical location
    ubyte2 canIdOutgoing;   // Calculated CAN ID for "setpoint" message
    ubyte2 canIdIncoming;   // Calculated CAN ID for "Actual1" message, also used for "Actual2" message

    int startUpStage;

    // Setpoints (commands, outgoing: 0x183 + offset)
    bool AMK_bInverterOn;
    bool AMK_bDcOn;
    bool AMK_bEnable;
    bool AMK_bErrorReset;
    sbyte2 AMK_TorqueSetpoint;
    sbyte2 AMK_TorqueLimitPositiv;
    sbyte2 AMK_TorqueLimitNegativ;

    // Actual Values 1 (incoming: 0x282 + offset)
    bool AMK_bSystemReady;
    bool AMK_bError;
    bool AMK_bWarn;
    bool AMK_bQuitDcOn;
    bool AMK_bDcOnVal;
    bool AMK_bQuitInverterOnVal;
    bool AMK_bInverterOnVal;
    bool AMK_bDerating;
    float4 AMK_ActualVelocity;  // RPM (SRE-7 Update: May Need Multiplier)
    float4 AMK_TorqueCurrent;
    float4 AMK_MagnetizingCurrent;

    // Actual Values 2 (incoming: 0x284 + offset)
    float4 AMK_TempMotor;       // 0.1degC
    float4 AMK_TempInverter;    // 0.1degC
    ubyte2 AMK_ErrorInfo;       
    float4 AMK_TorqueFeedback;        // Nm

} _DriveInverter;

_DriveInverter* AmkDriver_new(DI_Location_Address location_address);

void DI_calculateInverterControl(_DriveInverter* Idv, Sensor *HVILTermSense, TorqueEncoder *tps, BrakePressureSensor *bps, ReadyToDriveSound *rtds, _DAQSensors *d1);

void DI_calculateCommands(_DriveInverter* Idv, TorqueEncoder *tps, BrakePressureSensor *bps);

void DI_parseCanMessage(_DriveInverter* Idv, IO_CAN_DATA_FRAME* diCanMessage);

void DI_commandTorque(_DriveInverter* Idv, sbyte2 newTorque);

sbyte2 DI_getCommandedTorque(_DriveInverter* Idv);

// (for internal use)
static void DI_handleRelayOff(_DriveInverter* me);
static void DI_handleRelayOnSendingCan(_DriveInverter* me);
static void DI_handlePrechargeDcEnable(_DriveInverter* me, bool hvil_ok, _DAQSensors *d1);
static void DI_handleDriverEnable(_DriveInverter* me, TorqueEncoder *tps);
static void DI_handleReadyToDriveInverterOn(_DriveInverter* me, ReadyToDriveSound *rtds);
static void DI_handleTorqueLimitSet(_DriveInverter* me);
static void DI_handleTorqueSetpointsActive(_DriveInverter* me, bool hvil_ok);
static void DI_parseStatusBits(_DriveInverter* me, ubyte1 data);
static void DI_parseActualValues1(_DriveInverter* me, ubyte1* data);
static void DI_parseActualValues2(_DriveInverter* me, ubyte1* data);
static inline void DI_cmd(_DriveInverter* me, bool invOn, bool dcOn, bool enable, bool errReset, sbyte2 torque01Nm, ubyte2 limPos01Nm, ubyte2 limNeg01Nm);

#endif