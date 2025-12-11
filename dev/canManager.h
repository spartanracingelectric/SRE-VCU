/*****************************************************************************
 * canManager.h - CAN Message Sending and Recieve
 * Initial Author: Rusty P 
 ******************************************************************************
 * Works with sending the CAN messages and recieving with the specific values
 ****************************************************************************/

#ifndef _CANMANAGER_H
#define _CANMANAGER_H

#include "IO_Driver.h"
#include "IO_CAN.h"

#include "avlTree.h"
#include "instrumentCluster.h"
#include "bms.h"
#include "safety.h"
#include "AMKdrive.h"
#include "daqSensors.h"
//#include "sensorCalculations.h"

typedef enum
{
    CAN0_HIPRI,
    CAN1_LOPRI
} CanChannel;

#define CAN_CHANNELS  2
//CAN0: 48 messages per handle (48 read, 48 write)
//CAN1: 16 messages per handle

typedef struct _CanManager CanManager;

typedef struct _CanMessageNode CanMessageNode;

//Note: Sum of messageLimits must be < 128 (hardware only does 128 total messages)
CanManager *CanManager_new(ubyte2 busSpeed[CAN_CHANNELS], ubyte1 read_messageLimit[CAN_CHANNELS], ubyte1 write_messageLimit[CAN_CHANNELS], ubyte4 defaultSendDelayus);
IO_ErrorType CanManager_send(CanManager *me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount);

//Reads and distributes can messages to their appropriate subsystem objects so they can updates themselves
void CanManager_read(CanManager *me, CanChannel channel, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc, _DAQSensors *d1, _DriveInverter *inv1, _DriveInverter *inv2, _DriveInverter *inv3, _DriveInverter *inv4);

void canOutput_sendSensorMessages(CanManager *me);
//void canOutput_sendMCUControl(CanManager* me, MotorController* mcm, bool sendEvenIfNoChanges);
void canOutput_sendDebugMessage0(CanManager *me, TorqueEncoder *tps, BrakePressureSensor *bps, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc, _DriveInverter *inv1, _DriveInverter *inv2);
void canOutput_sendDebugMessage1(CanManager *me, _DriveInverter *inv1, _DriveInverter *inv2, _DriveInverter *inv3, _DriveInverter *inv4);

ubyte1 CanManager_getReadStatus(CanManager *me, CanChannel channel);

#endif // _CANMANAGER_H is defined