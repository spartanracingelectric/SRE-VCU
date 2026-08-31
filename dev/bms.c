/*****************************************************************************
 * bms.c - Battery Management System Parser
 * Initial Author: Rusty P / Vincent Saw
 * Additional Author: Akash Karthik
 ******************************************************************************
 * Deals with parsing from BMS and coordinating values in between
 ****************************************************************************/

#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "mathFunctions.h"

/*********************************************************
 *            *********** CAUTION ***********            *
 * MULTI-BYTE VALUES FOR THE 16's BMS ARE LITTLE-ENDIAN *
 *                                                       *
 *********************************************************/

//Pull a little-endian ubyte2 out of a frame starting at byte b ngl this was p useful
#define LE16(data, b) ( ((ubyte2)(data)[(b) + 1] << 8) | (ubyte2)(data)[(b)] )

struct _BatteryManagementSystem
{

    ubyte2 canMessageBaseId;

    //BMS Member Variable format:
    //byte(s), scaling, add'l comments

    // BMS_SAFETY_STATUS //
    ubyte1 warningFlags;                            //0
    ubyte1 faultFlags;                              //1
    ubyte2 cellMismatch;                            //3:2, V*1000
    ubyte4 packVoltage;                             //5:4, V*1000, HV sense
    ubyte4 sumOfCellVoltages;                       //7:6, V*1000

    // BMS_STATE_OF_CHARGE //
    ubyte2 ampHoursRemaining;                       //1:0, mAh
    ubyte1 stateOfCharge;                           //2, whole percent
    ubyte4 packCurrent;                             //6:3, A*1000, magnitude only cuz bms sends it unsigned

    // BMS_CELL_SUMMARY //
    ubyte2 highestCellVoltage;                      //1:0, V*1000
    ubyte2 lowestCellVoltage;                       //3:2, V*1000
    sbyte2 highestCellTemperature;                  //4, degC*10
    sbyte2 lowestCellTemperature;                   //5, degC*10

    // BMS_BALANCE_STATUS_1 and _2 //
    ubyte2 balanceStatus[BMS_NUM_MODULES];          //4 modules per frame, bitN=1 - shunting active for cell N+1

    // BMS_PRECHARGE_STATUS //
    bool prechargeComplete;                         //0

    // BMS_CELL_VOLTAGE_FIRST..LAST //
    ubyte2 cellVoltage[BMS_NUM_CELLS];              //4 cells per frame, V*1000

    // BMS_CELL_TEMPERATURE_FIRST..LAST //
    ubyte1 cellTemperature[BMS_NUM_THERMISTORS];    //degC
    ubyte1 modulePressure[BMS_NUM_MODULES];         //4
    ubyte1 moduleAtmosTemp[BMS_NUM_MODULES];        //5
    ubyte1 moduleHumidity[BMS_NUM_MODULES];         //6
    ubyte1 moduleDewPoint[BMS_NUM_MODULES];         //7

    ubyte4 timestamp_lastFaultFrame;  //IO_RTC time of last BMS_SAFETY_STATUS received

    bool relayState;
    bool prechargeRequest;  //what the VCU is asking for on BMS_PRECHARGE_COMMAND
};

BatteryManagementSystem *BMS_new(ubyte2 canMessageBaseID)
{
    ubyte1 i;

    BatteryManagementSystem *me = (BatteryManagementSystem *)malloc(sizeof(struct _BatteryManagementSystem));

    me->canMessageBaseId = canMessageBaseID;

    me->warningFlags = 0;
    me->faultFlags = 0xFF; //fail-safe: faulted until the first real safety frame clears it
    me->cellMismatch = 0;
    me->packVoltage = 0;
    me->sumOfCellVoltages = 0;

    me->ampHoursRemaining = 0;
    me->stateOfCharge = 0;
    me->packCurrent = 0;

    me->highestCellVoltage = 4000;
    me->lowestCellVoltage = 4000;
    me->highestCellTemperature = 0;
    me->lowestCellTemperature = 0;

    me->prechargeComplete = FALSE;
    me->prechargeRequest = FALSE;
    me->relayState = FALSE;
    me->timestamp_lastFaultFrame = 0;

    for (i = 0; i < BMS_NUM_CELLS; i++) { me->cellVoltage[i] = 0; }
    for (i = 0; i < BMS_NUM_THERMISTORS; i++) { me->cellTemperature[i] = 0; }
    for (i = 0; i < BMS_NUM_MODULES; i++)
    {
        me->balanceStatus[i] = 0;
        me->modulePressure[i] = 0;
        me->moduleAtmosTemp[i] = 0;
        me->moduleHumidity[i] = 0;
        me->moduleDewPoint[i] = 0;
    }

    return me;
}

void BMS_parseCanMessage(BatteryManagementSystem *bms, IO_CAN_DATA_FRAME *bmsCanMessage)
{
    ubyte1 *data = bmsCanMessage->data;
    ubyte2 offset;
    ubyte1 module;
    ubyte1 cell;
    ubyte1 i;

    if (bmsCanMessage->id < bms->canMessageBaseId)
    {
        return;
    }

    //Subtract BMS Base CAN ID from incoming BMS CAN message ID to get the offset
    //Ex: 0x622 - 0x600 = BMS_CELL_SUMMARY
    offset = bmsCanMessage->id - bms->canMessageBaseId;

    if (offset == BMS_SAFETY_STATUS)
    {
        bms->warningFlags       = data[0];
        bms->faultFlags         = data[1];
        bms->cellMismatch       = LE16(data, 2) / BMS_CELL_VOLTAGE_RAW_PER_MV;
        bms->packVoltage        = (ubyte4)LE16(data, 4) * BMS_PACK_VOLTAGE_MV_PER_RAW;
        bms->sumOfCellVoltages  = (ubyte4)LE16(data, 6) * BMS_PACK_VOLTAGE_MV_PER_RAW;
        IO_RTC_StartTime(&bms->timestamp_lastFaultFrame); //feeds BMS_isAlive
    }
    else if (offset == BMS_STATE_OF_CHARGE)
    {
        bms->ampHoursRemaining  = LE16(data, 0);
        bms->stateOfCharge      = data[2];
        bms->packCurrent        = ( ((ubyte4)data[6] << 24)
                                  | ((ubyte4)data[5] << 16)
                                  | ((ubyte4)data[4] << 8)
                                  | ((ubyte4)data[3])
                                  );
    }
    else if (offset == BMS_CELL_SUMMARY)
    {
        bms->highestCellVoltage     = LE16(data, 0) / BMS_CELL_VOLTAGE_RAW_PER_MV;
        bms->lowestCellVoltage      = LE16(data, 2) / BMS_CELL_VOLTAGE_RAW_PER_MV;
        bms->highestCellTemperature = (sbyte2)data[4] * BMS_TEMPERATURE_SCALE;
        bms->lowestCellTemperature  = (sbyte2)data[5] * BMS_TEMPERATURE_SCALE;
    }
    else if (offset == BMS_BALANCE_STATUS_1 || offset == BMS_BALANCE_STATUS_2)
    {
        module = (offset == BMS_BALANCE_STATUS_1) ? 0 : 4;
        for (i = 0; i < 4; i++)
        {
            bms->balanceStatus[module + i] = LE16(data, i * 2);
        }
    }
    else if (offset == BMS_PRECHARGE_STATUS)
    {
        bms->prechargeComplete = (data[0] == 0x02);
    }
    else if (offset >= BMS_CELL_VOLTAGE_FIRST && offset <= BMS_CELL_VOLTAGE_LAST)
    {
        cell = (offset - BMS_CELL_VOLTAGE_FIRST) * 4;
        for (i = 0; i < 4; i++)
        {
            bms->cellVoltage[cell + i] = LE16(data, i * 2) / BMS_CELL_VOLTAGE_RAW_PER_MV;
        }
    }
    else if (offset >= BMS_CELL_TEMPERATURE_FIRST && offset <= BMS_CELL_TEMPERATURE_LAST)
    {
        module = (offset - BMS_CELL_TEMPERATURE_FIRST) / 2;

        //First frame of the module carries thermistors 1-8
        if (((offset - BMS_CELL_TEMPERATURE_FIRST) % 2) == 0)
        {
            for (i = 0; i < 8; i++)
            {
                bms->cellTemperature[module * BMS_THERMISTORS_PER_MODULE + i] = data[i];
            }
        }
        //Second frame carries thermistors 9-12 and thene environmental stuff
        else
        {
            for (i = 0; i < 4; i++)
            {
                bms->cellTemperature[module * BMS_THERMISTORS_PER_MODULE + 8 + i] = data[i];
            }
            bms->modulePressure[module]  = data[4];
            bms->moduleAtmosTemp[module] = data[5];
            bms->moduleHumidity[module]  = data[6];
            bms->moduleDewPoint[module]  = data[7];
        }
    }
}

//1s covers the BMS's worst honest burst cadence (~120ms) plus its 500ms
//silent-drop window (can_skip_flag) without nuisance trips
#define BMS_RX_TIMEOUT_US 1000000

bool BMS_isAlive(BatteryManagementSystem *me)
{
    if (me->timestamp_lastFaultFrame == 0)
    {
        return FALSE; //never received a 0x600 since boot
    }
    return (IO_RTC_GetTimeUS(me->timestamp_lastFaultFrame) < BMS_RX_TIMEOUT_US);
}

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me)
{
    //////////////////////////////////////////////////////////////
    // Digital output to drive a signal to the Shutdown signal  //
    // based on AMS fault detection                             //
    //////////////////////////////////////////////////////////////
    IO_ErrorType err;
    //There is a fault, or the BMS has gone silent (treat silence as fault)
    // if (BMS_getFaultFlags(me) || BMS_isAlive(me) == FALSE)
    // {
    //     me->relayState = TRUE;
    //     err = IO_DO_Set(IO_DO_01, TRUE); //VCU pin 132, shutdown signal true (HIGH)
    // }
    // else
    // {
    //     me->relayState = FALSE;
    //     err = IO_DO_Set(IO_DO_01, FALSE); //VCU pin 132, shutdown signal false (LOW)
    // }
    // return err;
}

ubyte1 BMS_getFaultFlags(BatteryManagementSystem *me) {
    //Flag 0x04: Cell Over-Temperature Fault
    //Flag 0x08: Cell Voltage Imbalance Fault
    //Flag 0x10: Cell Under-Voltage Fault
    //Flag 0x20: Cell Over-Voltage Fault
    //Flag 0x40: Pack Under-Voltage Fault
    //Flag 0x80: Pack Over-Voltage Fault
    return me->faultFlags;
}

ubyte1 BMS_getWarningFlags(BatteryManagementSystem *me) {
    //Same bit layout as the fault byte, at the BMS's warning thresholds
    return me->warningFlags;
}

bool BMS_getRelayState(BatteryManagementSystem *me) {
    //Return state of shutdown board relay
    return me->relayState;
}

void BMS_updatePrechargeRequest(BatteryManagementSystem *me, Sensor *HVILTermSense)
{
    if (me->prechargeComplete == TRUE && HVILTermSense->sensorValue == FALSE)
    {
        me->prechargeRequest = FALSE;
    }
    else
    {
        me->prechargeRequest = TRUE;
    }
}

bool BMS_getPrechargeRequest(BatteryManagementSystem *me)
{
    return me->prechargeRequest;
}

ubyte4 BMS_getPackVoltage(BatteryManagementSystem *me)
{
    return (me->packVoltage);
}

ubyte4 BMS_getPackCurrent_mA(BatteryManagementSystem *me)
{
    return (me->packCurrent);
}

sbyte4 BMS_getPower_W(BatteryManagementSystem *me)
{
    return ((sbyte4)(me->packVoltage / BMS_VOLTAGE_SCALE) * (sbyte4)(me->packCurrent / BMS_CURRENT_SCALE));
}

ubyte1 BMS_getStateOfCharge(BatteryManagementSystem *me)
{
    return (me->stateOfCharge);
}

bool BMS_getPrechargeComplete(BatteryManagementSystem *me)
{
    return (me->prechargeComplete);
}

ubyte2 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->highestCellVoltage);
}

ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->lowestCellVoltage);
}

ubyte2 BMS_getCellMismatch_mV(BatteryManagementSystem *me)
{
    return (me->cellMismatch);
}

sbyte2 BMS_getHighestCellTemp_d_degC(BatteryManagementSystem *me)
{
    //Need to divide by BMS_TEMPERATURE_SCALE at usage to get deciCelsius value into Celsius
    return (me->highestCellTemperature);
}

sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem *me)
{
    return (me->highestCellTemperature / BMS_TEMPERATURE_SCALE);
}

sbyte2 BMS_getLowestCellTemp_degC(BatteryManagementSystem *me)
{
    return (me->lowestCellTemperature / BMS_TEMPERATURE_SCALE);
}

ubyte2 BMS_getCellVoltage_mV(BatteryManagementSystem *me, ubyte1 cell)
{
    return (cell < BMS_NUM_CELLS) ? me->cellVoltage[cell] : 0;
}

ubyte1 BMS_getCellTemp_degC(BatteryManagementSystem *me, ubyte1 thermistor)
{
    return (thermistor < BMS_NUM_THERMISTORS) ? me->cellTemperature[thermistor] : 0;
}

ubyte2 BMS_getBalanceStatus(BatteryManagementSystem *me, ubyte1 module)
{
    return (module < BMS_NUM_MODULES) ? me->balanceStatus[module] : 0;
}

ubyte1 BMS_getModulePressure(BatteryManagementSystem *me, ubyte1 module)
{
    return (module < BMS_NUM_MODULES) ? me->modulePressure[module] : 0;
}

ubyte1 BMS_getModuleAtmosTemp(BatteryManagementSystem *me, ubyte1 module)
{
    return (module < BMS_NUM_MODULES) ? me->moduleAtmosTemp[module] : 0;
}

ubyte1 BMS_getModuleHumidity(BatteryManagementSystem *me, ubyte1 module)
{
    return (module < BMS_NUM_MODULES) ? me->moduleHumidity[module] : 0;
}

ubyte1 BMS_getModuleDewPoint(BatteryManagementSystem *me, ubyte1 module)
{
    return (module < BMS_NUM_MODULES) ? me->moduleDewPoint[module] : 0;
}
