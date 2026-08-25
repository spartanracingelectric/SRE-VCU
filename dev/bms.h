/*****************************************************************************
 * bms.h - Battery Management System Parser
 * Initial Author: Rusty P / Vincent Saw
 * Additional Author: Akash Karthik
 ******************************************************************************
 * Deals with parsing from BMS and coordinating values in between
 *
 * Uses 16's BMS
 ****************************************************************************/

#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H

#include <stdio.h>
#include <stdint.h>

#include "IO_CAN.h"
#include "sensors.h"    //Sensor, for the HVIL termination sense passed to the precharge request

//Max mismatch voltage, in volts
//To determine VCU-side fault
#define BMS_MAX_CELL_MISMATCH_V 1.00f
#define BMS_MIN_CELL_VOLTAGE_WARNING 3.20f
#define BMS_MAX_CELL_TEMPERATURE_WARNING 55.0f

// Pack layout
#define BMS_NUM_MODULES                 8
#define BMS_CELLS_PER_MODULE            12
#define BMS_THERMISTORS_PER_MODULE      12
#define BMS_NUM_CELLS                   (BMS_NUM_MODULES * BMS_CELLS_PER_MODULE)
#define BMS_NUM_THERMISTORS             (BMS_NUM_MODULES * BMS_THERMISTORS_PER_MODULE)

/////////////////////////////////////////////////////////////////////
// CAN PROTOCOL CONSTANTS, offsets from base address               //
// Ex: canMessageBaseId + BMS_CELL_SUMMARY = Address 0x600 + 0x022 //
/////////////////////////////////////////////////////////////////////

#define BMS_BASE_ADDRESS                0x600

// BMS Received Messages (VCU --> BMS)
#define BMS_BALANCE_COMMAND             0x004   //1 byte, data[0]: 1 means balancing is on
#define BMS_PRECHARGE_COMMAND           0x005   //1 byte, data[0]: 1 means close the precharge relay
//NOTE: 0x605 belongs to the precharge command. Do not reuse it for debug frames -
//the BMS acts on it, and it also falls inside the VCU's own BMS receive range

// BMS Transmitted Messages (BMS --> VCU)
#define BMS_SAFETY_STATUS               0x000   //8 bytes
#define BMS_STATE_OF_CHARGE             0x021   //7 bytes
#define BMS_CELL_SUMMARY                0x022   //6 bytes
#define BMS_BALANCE_STATUS_1            0x023   //8 bytes, modules 1-4
#define BMS_BALANCE_STATUS_2            0x024   //8 bytes, modules 5-8
#define BMS_PRECHARGE_STATUS            0x025   //1 byte
#define BMS_CELL_VOLTAGE_FIRST          0x030   //8 bytes each, 4 cells per frame
#define BMS_CELL_VOLTAGE_LAST           (BMS_CELL_VOLTAGE_FIRST + (BMS_NUM_CELLS / 4) - 1)
#define BMS_CELL_TEMPERATURE_FIRST      0x080   //8 bytes each, 2 frames per module
#define BMS_CELL_TEMPERATURE_LAST       (BMS_CELL_TEMPERATURE_FIRST + (BMS_NUM_MODULES * 2) - 1)
#define BMS_LAST_ADDRESS                BMS_CELL_TEMPERATURE_LAST

// BMS Scaling factors
// X/SCALE
#define BMS_VOLTAGE_SCALE               1000    //V*1000
#define BMS_CURRENT_SCALE               1000    //A*1000
#define BMS_TEMPERATURE_SCALE           10      //degC*10

#define BMS_CELL_VOLTAGE_RAW_PER_MV     10      //cells arrive in 100uV incremebts
#define BMS_PACK_VOLTAGE_MV_PER_RAW     10      //pack voltage arrives in 10mV increments

//Fault/warning bits of the BMS_SAFETY_STATUS frame (fault byte 1, warning byte 0)
//NOTE: the BMS firmware never sets MISMATCH or the PACK_ bits (dead code there) -
//the VCU must not claim coverage of those conditions
#define BMS_CELL_OVER_TEMPERATURE_FLAG  0x04
#define BMS_CELL_MISMATCH_FLAG          0x08
#define BMS_CELL_UNDER_VOLTAGE_FLAG     0x10
#define BMS_CELL_OVER_VOLTAGE_FLAG      0x20
#define BMS_PACK_UNDER_VOLTAGE_FLAG     0x40
#define BMS_PACK_OVER_VOLTAGE_FLAG      0x80

typedef struct _BatteryManagementSystem BatteryManagementSystem;

BatteryManagementSystem* BMS_new(ubyte2 canMessageBaseID);
void BMS_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage);
bool BMS_isAlive(BatteryManagementSystem *me);

// BMS COMMANDS //

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me);
bool BMS_getRelayState(BatteryManagementSystem *me);

//Recalculates whether the VCU is asking the BMS to precharge. Call once per main loop,
//after the CAN read, and before the frame is put on the bus
void BMS_updatePrechargeRequest(BatteryManagementSystem *me, Sensor *HVILTermSense);
bool BMS_getPrechargeRequest(BatteryManagementSystem *me);

// Pack level //

ubyte1 BMS_getFaultFlags(BatteryManagementSystem *me);
ubyte1 BMS_getWarningFlags(BatteryManagementSystem *me);
ubyte4 BMS_getPackVoltage(BatteryManagementSystem *me);             //Millivolts
ubyte4 BMS_getPackCurrent_mA(BatteryManagementSystem *me);          //Milliamps
sbyte4 BMS_getPower_W(BatteryManagementSystem *me);                 //Watts
ubyte1 BMS_getStateOfCharge(BatteryManagementSystem *me);           //Percent
bool BMS_getPrechargeComplete(BatteryManagementSystem *me);

// Cell summary //

ubyte2 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me);
ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me);
ubyte2 BMS_getCellMismatch_mV(BatteryManagementSystem *me);
sbyte2 BMS_getHighestCellTemp_d_degC(BatteryManagementSystem *me);  //deciCelsius
sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem *me);    //Celsius
sbyte2 BMS_getLowestCellTemp_degC(BatteryManagementSystem *me);     //Celsius

// Per cell and per module, all return 0 if the index is out of range //

ubyte2 BMS_getCellVoltage_mV(BatteryManagementSystem *me, ubyte1 cell);
ubyte1 BMS_getCellTemp_degC(BatteryManagementSystem *me, ubyte1 thermistor);
ubyte2 BMS_getBalanceStatus(BatteryManagementSystem *me, ubyte1 module);
ubyte1 BMS_getModulePressure(BatteryManagementSystem *me, ubyte1 module);
ubyte1 BMS_getModuleAtmosTemp(BatteryManagementSystem *me, ubyte1 module);
ubyte1 BMS_getModuleHumidity(BatteryManagementSystem *me, ubyte1 module);
ubyte1 BMS_getModuleDewPoint(BatteryManagementSystem *me, ubyte1 module);

#endif // _BATTERYMANAGEMENTSYSTEM_H
