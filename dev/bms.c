
#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "serial.h"
#include "mathFunctions.h"

/*********************************************************
 *            *********** CAUTION ***********            *
 * MULTI-BYTE VALUES FOR THE STAFL BMS ARE LITTLE-ENDIAN *
 *                                                       *
 *********************************************************/

/**** MOVED STRUCT TO BMS.H ****/


BatteryManagementSystem *BMS_new(SerialManager *serialMan, ubyte2 canMessageBaseID, BMSPack pack)
{

    BatteryManagementSystem *me = (BatteryManagementSystem *)malloc(sizeof(struct _BatteryManagementSystem));

    me->canMessageBaseId = canMessageBaseID;
    me->sm = serialMan;
    //me->maxTemp = 99;

    me->packCurrent = 0;
    me->packVoltage = 0;
    //Repick a new value, maybe 0xFFFF?
    me->highestCellVoltage = 0;
    me->lowestCellVoltage = 0;
    me->highestCellTemperature = 0;
	me->averageCellTemperature = 0;

    me->faultFlags0 = 0;
    me->faultFlags1 = 0;
    //me->faultFlags0 = 0xFF;
    //me->faultFlags1 = 0xFF;

    me->relayState = FALSE;

	me->pack = pack;

    return me;
}

void BMS_parseCanMessage(BatteryManagementSystem *bms, IO_CAN_DATA_FRAME *bmsCanMessage)
{
	if (bms->pack == BMS_PACK_SR17) 
	{
		switch (bmsCanMessage->id)
		{
			case BMS_PACK_SUMMARY_ONE_CAN_ID:
				bms->highestCellVoltage = (ubyte2)(((ubyte2)bmsCanMessage->data[1] << 8) | ((ubyte2)bmsCanMessage->data[0]));
				bms->lowestCellVoltage = (ubyte2)(((ubyte2)bmsCanMessage->data[3] << 8) | ((ubyte2)bmsCanMessage->data[2]));
				bms->averageCellTemperature = (sbyte1)bmsCanMessage->data[4];
				bms->highestCellTemperature = (sbyte1)bmsCanMessage->data[5];
				bms->packVoltage = (ubyte2)(((ubyte2)bmsCanMessage->data[7] << 8) | ((ubyte2)bmsCanMessage->data[6]));
				break;
		}
	}
	else if (bms->pack == BMS_PACK_SR16)
	{
		switch (bmsCanMessage->id)
		{
			case BMS_PACK_SUMMARY_ONE_CAN_ID:
				bms->highestCellVoltage = (ubyte2)(((ubyte2)bmsCanMessage->data[1] << 8) | ((ubyte2)bmsCanMessage->data[0]));
				bms->lowestCellVoltage = (ubyte2)(((ubyte2)bmsCanMessage->data[3] << 8) | ((ubyte2)bmsCanMessage->data[2]));
				bms->highestCellTemperature = (ubyte1)bmsCanMessage->data[4];
				bms->lowestCellTemperature = (ubyte1)bmsCanMessage->data[5];
				break;
        }
    }
}

IO_ErrorType BMS_relayControl(BatteryManagementSystem *me)
{
    //////////////////////////////////////////////////////////////
    // Digital output to drive a signal to the Shutdown signal  //
    // based on AMS fault detection                             //
    //////////////////////////////////////////////////////////////
    IO_ErrorType err;
    //There is a fault
    if (BMS_getFaultFlags0(me) || BMS_getFaultFlags1(me))
    {
        me->relayState = TRUE;
        err = IO_DO_Set(IO_DO_01, TRUE); //Drive BMS relay true (HIGH)
    }
    //There is no fault
    else
    {
        me->relayState = FALSE;
        err = IO_DO_Set(IO_DO_01, FALSE); //Drive BMS relay false (LOW)
    }
    return err;
}

/*
sbyte1 BMS_getAvgTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "AvgPackTemp: %i\n", me->avgTemp);
    return (me->avgTemp);
}
*/

ubyte4 BMS_getHighestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->highestCellVoltage);
}

ubyte2 BMS_getLowestCellVoltage_mV(BatteryManagementSystem *me)
{
    return (me->lowestCellVoltage);
}

ubyte4 BMS_getPackVoltage_cV(BatteryManagementSystem *me)
{
    return (me->packVoltage); 
}

sbyte2 BMS_getHighestCellTemp_degC(BatteryManagementSystem *me)
{
    return (me->highestCellTemperature);
}

sbyte2 BMS_getAverageCellTemp_degC(BatteryManagementSystem *me)
{
	return (me->averageCellTemperature);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_uW(BatteryManagementSystem *me)
{
    //char buffer[32];
    //sprintf(buffer, "power (uW): %f\n", (me->packCurrent * me->packVoltage));

    //Need to divide by BMS_POWER_SCALE at usage to get microWatt value into Watts
    return (me->packCurrent * me->packVoltage);
}

// ***NOTE: packCurrent and and packVoltage are SIGNED variables and the return type for BMS_getPower is signed
sbyte4 BMS_getPower_W(BatteryManagementSystem *me)
{
    //char buffer[32];
    //sprintf(buffer, "power (W): %f\n", ((me->packCurrent * me->packVoltage)/BMS_POWER_SCALE));

    //Need to divide by BMS_POWER_SCALE at usage to get microWatt value into Watts
    return ((me->packCurrent * me->packVoltage)/BMS_POWER_SCALE);
}

ubyte1 BMS_getFaultFlags0(BatteryManagementSystem *me) {
    //Flag 0x01: Isolation Leakage Fault
    //Flag 0x02: BMS Monitor Communication Fault
    //Flag 0x04: Pre-charge Fault
    //Flag 0x08: Pack Discharge Operating Envelope Exceeded
    //Flag 0x10: Pack Charge Operating Envelope Exceeded
    //Flag 0x20: Failed Thermistor Fault
    //Flag 0x40: HVIL Fault
    //Flag 0x80: Emergency Stop Fault
    return me->faultFlags0;
}

ubyte1 BMS_getFaultFlags1(BatteryManagementSystem *me) {
    //Flag 0x01: Cell Over-Voltage Fault
    //Flag 0x02: Cell Under-Voltage Fault
    //Flag 0x04: Cell Over-Temperature Fault
    //Flag 0x08: Cell Under-Temperature Fault
    //Flag 0x10: Pack Over-Voltage Fault
    //Flag 0x20: Pack Under-Voltage Fault
    //Flag 0x40: Over-Current Discharge Fault
    //Flag 0x80: Over-Current Charge Fault
    return me->faultFlags1;
}

bool BMS_getRelayState(BatteryManagementSystem *me) {
    //Return state of shutdown board relay
    return me->relayState;
}

/*
ubyte2 BMS_getPackTemp(BatteryManagementSystem *me)
{
    char buffer[32];
    sprintf(buffer, "PackTemp: %i\n", me->packTemp);
    return (me->packTemp);
}
*/
