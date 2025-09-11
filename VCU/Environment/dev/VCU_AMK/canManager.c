
/*****************************************************************************
 * canManager.c - CAN Message Sending and Recieve
 * Initial Author: Rusty P 
 ******************************************************************************
 * Works with sending the CAN messages and recieving with the specific values
 ****************************************************************************/

#include <stdlib.h> //malloc

#include "IO_Driver.h" 
#include "IO_CAN.h"
#include "IO_RTC.h"

#include "mathFunctions.h"
#include "sensors.h"
#include "canManager.h"
#include "avlTree.h"
#include "bms.h"
#include "safety.h"
#include "sensorCalculations.h"
#include "daqSensors.h"


struct _CanManager {

    ubyte1 canMessageLimit;
    
    ubyte1 busSpeed[CAN_CHANNELS];
    ubyte1 readHandle[CAN_CHANNELS];
    ubyte1 read_messageLimit[CAN_CHANNELS];
    ubyte1 writeHandle[CAN_CHANNELS];
    ubyte1 write_messageLimit[CAN_CHANNELS];
    
    IO_ErrorType ioErr_Init[CAN_CHANNELS];

    IO_ErrorType ioErr_fifoInit_R[CAN_CHANNELS];
    IO_ErrorType ioErr_fifoInit_W[CAN_CHANNELS];

    IO_ErrorType ioErr_read[CAN_CHANNELS];
    IO_ErrorType ioErr_write[CAN_CHANNELS];

    ubyte4 sendDelayus;

    AVLNode* canMessageHistory[0x7FF];
};

CanManager* CanManager_new(ubyte2 busSpeed[CAN_CHANNELS], ubyte1 read_messageLimit[CAN_CHANNELS], ubyte1 write_messageLimit[CAN_CHANNELS], ubyte4 defaultSendDelayus) //ubyte4 defaultMinSendDelay, ubyte4 defaultMaxSendDelay)
{
    CanManager* me = (CanManager*)malloc(sizeof(struct _CanManager));

    for (ubyte4 id = 0; id <= 0x7FF; id++)
    {
        me->canMessageHistory[id] = 0;
    }

    me->sendDelayus = defaultSendDelayus;

    for (int i=0; i < 2; ++ i){
        //Activate the CAN channels --------------------------------------------------
        me->ioErr_Init[i] = IO_CAN_Init(IO_CAN_CHANNEL_0+i, busSpeed[i], 0, 0, 0);

        //Configure the FIFO queues
        //This specifies: The handle names for the queues
        //, which channel the queue belongs to
        //, the # of messages (or maximum count?)
        //, the direction of the queue (in/out)
        //, the frame size
        IO_CAN_ConfigFIFO(&me->readHandle[i], IO_CAN_CHANNEL_0 + i, read_messageLimit[i], IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
        IO_CAN_ConfigFIFO(&me->writeHandle[i], IO_CAN_CHANNEL_0 + i, write_messageLimit[i], IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

        //Assume read/write at error state until used
        me->ioErr_read[i] = IO_E_CAN_BUS_OFF;
        me->ioErr_write[i] = IO_E_CAN_BUS_OFF;
    }
    //-------------------------------------------------------------------
    //Define default messages
    //-------------------------------------------------------------------
    ubyte2 messageID[CAN_CHANNELS] = {0x500,0x515};

    for (ubyte1 number = 0; messageID[0]+ number < messageID[1]; ++number){
        me->canMessageHistory[messageID[0] + number]->timeBetweenMessages_Min = 50000;
        me->canMessageHistory[messageID[0] + number]->timeBetweenMessages_Max = 250000;
        me->canMessageHistory[messageID[0] + number]->required = TRUE;
        for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[messageID[0] + number]->data[i] = 0; }
        IO_RTC_StartTime(&me->canMessageHistory[messageID[0] + number]->lastMessage_timeStamp);
    }

    ubyte2 amkMessageID[4] = {184,185,188,189};

    for(ubyte1 number = 0; number < 4; ++number){
        me->canMessageHistory[amkMessageID[number]]->timeBetweenMessages_Min = 0; // us (microseconds)
        me->canMessageHistory[amkMessageID[number]]->timeBetweenMessages_Max = 8000; 
        me->canMessageHistory[amkMessageID[number]]->required = TRUE;
        for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[amkMessageID[number]]->data[i] = 0; }
        IO_RTC_StartTime(&me->canMessageHistory[amkMessageID[number]]->lastMessage_timeStamp);
    }

    return me;
}


/*****************************************************************************
* This function takes an array of messages, determines which messages to send
* based on whether or not data has changed since the last time it was sent,
* or if a certain amount of time has passed since the last time it was sent.
*
* Messages that need to be sent are copied to another array and passed to the
* FIFO queue.
*
* Note: http://stackoverflow.com/questions/5573310/difference-between-passing-array-and-array-pointer-into-function-in-c
* http://stackoverflow.com/questions/2360794/how-to-pass-an-array-of-struct-using-pointer-in-c-c
****************************************************************************/
IO_ErrorType CanManager_send(CanManager* me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount)
{
    //SerialManager_send(me->sm, "Do you even send?\n");
    bool sendSerialDebug = FALSE;
    ubyte2 serialMessageID = 0xC0;
    bool sendMessage = FALSE;
    ubyte1 messagesToSendCount = 0;
    IO_CAN_DATA_FRAME messagesToSend[canMessageCount];//[channel == CAN0_HIPRI ? me->can0_write_messageLimit : me->can1_write_messageLimit];

    //----------------------------------------------------------------------------
    // Check if message exists in outgoing message history tree
    //----------------------------------------------------------------------------
    AVLNode* lastMessage;  //replace with me->canMessageHistory[ID]
    ubyte1 messagePosition; //used twice
    for (messagePosition = 0; messagePosition < canMessageCount; messagePosition++)
    {
        bool firstTimeMessage = FALSE;
        bool dataChanged = FALSE;
        bool minTimeExceeded = FALSE;
        bool maxTimeExceeded = FALSE;

        ubyte2 outboundMessageID = canMessages[messagePosition].id;
        lastMessage = me->canMessageHistory[outboundMessageID];
        sendMessage = FALSE;

        //----------------------------------------------------------------------------
        // Check if this message exists in the array
        //----------------------------------------------------------------------------
        firstTimeMessage = (me->canMessageHistory[outboundMessageID] == 0);  
        if (firstTimeMessage)
        {
            me->canMessageHistory[outboundMessageID]->timeBetweenMessages_Min = 25000;
            me->canMessageHistory[outboundMessageID]->timeBetweenMessages_Max = 125000;
            me->canMessageHistory[outboundMessageID]->required = TRUE;
            for (ubyte1 i = 0; i <= 7; i++) { me->canMessageHistory[outboundMessageID]->data[i] = 0; }
            //IO_RTC_StartTime(&me->canMessageHistory[outboundMessageID]->lastMessage_timeStamp);
            me->canMessageHistory[outboundMessageID]->lastMessage_timeStamp = 0;
        }

        //----------------------------------------------------------------------------
        // Check if data has changed since last time message was sent
        //----------------------------------------------------------------------------
        //Check each data byte in the data array
        for (ubyte1 dataPosition = 0; dataPosition < 8; dataPosition++)
        {
            ubyte1 oldData = lastMessage->data[dataPosition];
            ubyte1 newData = canMessages[messagePosition].data[dataPosition];
            //if any data byte is changed, then probably want to send the message
            dataChanged = (oldData == newData) ? FALSE : TRUE; //Only want to send if dataChanged is true
        }//end checking each byte in message

        //----------------------------------------------------------------------------
        // Check if time has exceeded
        //----------------------------------------------------------------------------
        minTimeExceeded = ((IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= lastMessage->timeBetweenMessages_Min));
        maxTimeExceeded = ((IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= 50000));//lastMessage->timeBetweenMessages_Max));
        
        //----------------------------------------------------------------------------
        // If any criteria were exceeded, send the message out
        //----------------------------------------------------------------------------
        if (  (firstTimeMessage)
           || (dataChanged && minTimeExceeded)
           || (!dataChanged && maxTimeExceeded)
           )
        {
            sendMessage = TRUE;
        }

        //----------------------------------------------------------------------------
        // If we determined that this message should be sent
        //----------------------------------------------------------------------------
        if (sendMessage == TRUE)
        {
            //copy the message that needs to be sent into the outgoing messages array
            //see http://stackoverflow.com/questions/1693853/copying-arrays-of-structs-in-c
            //http://www.socialledge.com/sjsu/index.php?title=ES101_-_Lesson_9_:_Structures
            messagesToSend[messagesToSendCount++] = canMessages[messagePosition];
        }
        else
        {
            if (sendSerialDebug && (serialMessageID == outboundMessageID)) {
                //SerialManager_send(me->sm, "This message did not need to be sent.\n");
            }
        }
    } //end of loop for each message in outgoing messages

    IO_UART_Task();
    //----------------------------------------------------------------------------
    // If there are messages to send
    //----------------------------------------------------------------------------
    IO_ErrorType sendResult = IO_E_OK;
    if (messagesToSendCount > 0)
    {
        //Send the messages to send to the appropriate FIFO queue
        sendResult = IO_CAN_WriteFIFO((channel == CAN0_HIPRI) ? me->writeHandle[0] : me->writeHandle[1], messagesToSend, messagesToSendCount);
        *((channel == CAN0_HIPRI) ? &me->ioErr_write[0] : &me->ioErr_write[1]) = sendResult;

        //Update the outgoing message tree with message sent timestamps
        if ((channel == CAN0_HIPRI ? me->ioErr_write[0] : me->ioErr_write[1]) == IO_E_OK)
        {
            //Loop through the messages that we sent...
            for (messagePosition = 0; messagePosition < messagesToSendCount; messagePosition++)
            {
                // update the message sent timestamp
                IO_RTC_StartTime(&me->canMessageHistory[messagesToSend[messagePosition].id]->lastMessage_timeStamp);
            }
        }
    }
    return sendResult;
}


/*****************************************************************************
* read
****************************************************************************/
void CanManager_read(CanManager *me, CanChannel channel, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc, _DAQSensors *d1, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[(channel == CAN0_HIPRI ? me->read_messageLimit[0] : me->read_messageLimit[1])];
    ubyte1 canMessageCount;  //FIFO queue only holds 128 messages max

    //Read messages from hi/lopri channel 
    *(channel == CAN0_HIPRI ? &me->ioErr_read[0] : &me->ioErr_read[1]) =
    IO_CAN_ReadFIFO((channel == CAN0_HIPRI ? me->readHandle[0] : me->readHandle[0])
                    , canMessages
                    , (channel == CAN0_HIPRI ? me->read_messageLimit[0] : me->read_messageLimit[0])
                    , &canMessageCount);

    //Determine message type based on ID
    for (int currMessage = 0; currMessage < canMessageCount; currMessage++)
    {
        // Seperate based on CAN message
        switch (canMessages[currMessage].id)
        {
        //-------------------------------------------------------------------------
        //Inverters (Inverter FL and FR are together CAN0 and Inverter RL and RR are together CAN1) 
        //This is to ensure better debug between the two busses
        //-------------------------------------------------------------------------
        case 0x283:
        case 0x285:
            //Inverter FL 1 (CAN0)
            DI_parseCanMessage(inv1, &canMessages[currMessage]);
            break;
        case 0x284:
        case 0x286:
            //Inverter FR 1 (CAN0)
            DI_parseCanMessage(inv2, &canMessages[currMessage]);
            break;
        case 0x287:
        case 0x289:
            //Inverter RL 1 (CAN1)
            DI_parseCanMessage(inv1, &canMessages[currMessage]);
            break;
        case 0x288:
        case 0x290:
            //Inverter RR 1 (CAN1)
            DI_parseCanMessage(inv2, &canMessages[currMessage]);
            break;
        
        //-------------------------------------------------------------------------
        //IMU from DAQ
        //-------------------------------------------------------------------------
        case 0x400:
            DAQ_parseCanMessage(d1, &canMessages[currMessage]);
            break;
        case 0x401:
            DAQ_parseCanMessage(d1, &canMessages[currMessage]);
            break;
        case 0x402:
            DAQ_parseCanMessage(d1, &canMessages[currMessage]);
            break;
        case 0x403:
            DAQ_parseCanMessage(d1, &canMessages[currMessage]);
            break;

        //-------------------------------------------------------------------------
        //BMS
        //-------------------------------------------------------------------------
        case 0x600:
        case 0x602: //Faults
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x604:
        case 0x608:
        case 0x610:
        case 0x611:
        case 0x612:
        case 0x613:
        case 0x620:
        case 0x621:
        case 0x622: //Cell Voltage Summary
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x623: //Cell Temperature Summary
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;
        case 0x624:
        //1st Module
        case 0x630:
        case 0x631:
        case 0x632:
        //2nd Module
        case 0x633:
        case 0x634:
        case 0x635:
        //3rd Module
        case 0x636:
        case 0x637:
        case 0x638:
        //4th Module
        case 0x639:
        case 0x63A:
        case 0x63B:
        //5th Module
        case 0x63C:
        case 0x63D:
        case 0x63E:
        //6th Module
        case 0x63F:
        case 0x640:
        case 0x641:

        case 0x629:
            BMS_parseCanMessage(bms, &canMessages[currMessage]);
            break;

        case 0x702:
            //Need Updating: IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        case 0x703:
            //Need Updating: IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
        case 0x704:
            //Need Updating: IC_parseCanMessage(ic, mcm, &canMessages[currMessage]);
            break;
      
        //-------------------------------------------------------------------------
        //VCU Debug Control
        //-------------------------------------------------------------------------
        case 0x5FF:
            SafetyChecker_parseCanMessage(sc, &canMessages[currMessage]);
            //MCM_parseCanMessage(mcm, &canMessages[currMessage]);
            break;
            //default:
        }

        //Parse IMU here
    }
    //IO_CAN_WriteFIFO(me->can1_writeHandle, canMessages, messagesReceived);
    //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
}

ubyte1 CanManager_getReadStatus(CanManager* me, CanChannel channel)
{
    return (channel == CAN0_HIPRI) ? me->ioErr_read[0] : me->ioErr_read[1];
}




/*****************************************************************************
* device-specific functions
****************************************************************************/
/*****************************************************************************
* Standalone Sensor messages
******************************************************************************
* Load sensor values into CAN messages
* Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
* The message addresses are at:
* https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
****************************************************************************/
void canOutput_sendSensorMessages(CanManager* me)
{

}


//----------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------
void canOutput_sendDebugMessage0(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, InstrumentCluster* ic, BatteryManagementSystem* bms, SafetyChecker* sc, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[me->write_messageLimit[0]];
    ubyte1 errorCount;
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;
    ubyte1 byteNum;

    ubyte1 tps0Percent = 0xFF * tps->tps0_percent;  //Pedal percent int   (a number from 0 to 100)
    ubyte1 tps1Percent = 0xFF * tps->tps1_percent;
    ubyte1 throttlePercent = 0xFF * tps->travelPercent;
    ubyte1 brakePercent = 0xFF * bps->percent;
    
    //500: TPS 0
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1; //500
    canMessages[canMessageCount - 1].data[byteNum++] = throttlePercent;
    canMessages[canMessageCount - 1].data[byteNum++] = tps0Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_TPS0.sensorValue; // tps->tps0_value;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_TPS0.sensorValue >> 8; //tps->tps0_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //TPS 1
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = throttlePercent;
    canMessages[canMessageCount - 1].data[byteNum++] = tps1Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_value;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //BPS0
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = brakePercent; //This should be bps0Percent, but for now bps0Percent = brakePercent
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_value;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //WSS mm/s output //UNUSED
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //WSS RPM non-interpolated output //UNUSED
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    /*
    //TEMP: WSS3
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;  //505
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue >> 16;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RL.sensorValue >> 24;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue >> 16;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_RR.sensorValue >> 24;
    canMessages[canMessageCount - 1].length = byteNum;
    */
    /*
    //TEMP: FRONT WSS PIN DEBUG
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;  //505
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalInit;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalInit >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalGet;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FL.ioErr_signalGet >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalInit;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalInit >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalGet;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_WSS_FR.ioErr_signalGet >> 8;
    canMessages[canMessageCount - 1].length = byteNum;
    */

    //WSS RPM interpolated output
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //506: Safety Checker
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc) >> 16;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getFaults(sc) >> 24;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getWarnings(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getWarnings(sc) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getNotices(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getNotices(sc) >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //12v battery
    float4 LVBatterySOC = 0;
    if (Sensor_LVBattery.sensorValue < 12730)
        LVBatterySOC = .0 + .1 * getPercent(Sensor_LVBattery.sensorValue, 9200, 12730, FALSE);
    else if (Sensor_LVBattery.sensorValue < 12866)
        LVBatterySOC = .1 + .1 * getPercent(Sensor_LVBattery.sensorValue, 12730, 12866, FALSE);
    else if (Sensor_LVBattery.sensorValue < 12996)
        LVBatterySOC = .2 + .1 * getPercent(Sensor_LVBattery.sensorValue, 12866, 12996, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13104)
        LVBatterySOC = .3 + .1 * getPercent(Sensor_LVBattery.sensorValue, 12996, 13104, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13116)
        LVBatterySOC = .4 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13104, 13116, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13130)
        LVBatterySOC = .5 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13116, 13130, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13160)
        LVBatterySOC = .6 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13130, 13160, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13270)
        LVBatterySOC = .7 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13160, 13270, FALSE);
    else if (Sensor_LVBattery.sensorValue < 13300)
        LVBatterySOC = .8 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13270, 13300, FALSE);
    else //if (Sensor_LVBattery.sensorValue < 14340)
        LVBatterySOC = .9 + .1 * getPercent(Sensor_LVBattery.sensorValue, 13300, 14340, FALSE);

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte1)Sensor_LVBattery.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_LVBattery.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (sbyte1)(100 * LVBatterySOC);
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //508: Regen settings (Need to be updated for the future)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenMode(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getMaxTorqueDNm(mcm)/10;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenTorqueLimitDNm(mcm)/10;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenTorqueAtZeroPedalDNm(mcm)/10;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenAPPSForMaxCoastingZeroToFF(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getRegenBPSForMaxRegenZeroToFF(mcm);
    canMessages[canMessageCount - 1].length = byteNum;

    //509: MCM RTD Status
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_HVILTerminationSense.sensorValue;
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_HVILTerminationSense.sensorValue >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50A: Torque Vectoring Loopback (Future Needs)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50B: AMK VCU Debug
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = inv1->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = inv2->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50C: SAS (Steering Angle Sensor)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = steering_degrees();
    canMessages[canMessageCount - 1].data[byteNum++] = steering_degrees() >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50D: BPS1 (TEMPORARY ADDRESS)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = brakePercent; //This should be bps0Percent, but for now bps0Percent = brakePercent
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_value;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps1_calibMax >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    
    //50E: BMS Loopback Test
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getFaultFlags0(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getFaultFlags1(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getRelayState(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = BMS_getHighestCellTemp_d_degC(bms);
    canMessages[canMessageCount - 1].data[byteNum++] = (BMS_getHighestCellTemp_d_degC(bms) >> 8);
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //50F: Power Debug (Need adaption in the future)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //MCM_getPower(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //(MCM_getPower(mcm) >> 8);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //(MCM_getPower(mcm) >> 16);
    canMessages[canMessageCount - 1].data[byteNum++] = 0; //(MCM_getPower(mcm) >> 24);
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getWarnings(sc);
    canMessages[canMessageCount - 1].data[byteNum++] = (SafetyChecker_getWarnings(sc) >> 8);
    canMessages[canMessageCount - 1].data[byteNum++] = (SafetyChecker_getWarnings(sc) >> 16);
    canMessages[canMessageCount - 1].data[byteNum++] = (SafetyChecker_getWarnings(sc) >> 24);
    canMessages[canMessageCount - 1].length = byteNum;

    //511: SoftBSPD
    // ubyte1 flags = sc->softBSPD_bpsHigh;
    // flags |= sc->softBSPD_kwHigh << 1;
    // canMessageCount++;
    // byteNum = 0;
    // canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    // canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    // canMessages[canMessageCount - 1].data[byteNum++] = sc->softBSPD_fault;
    // canMessages[canMessageCount - 1].data[byteNum++] = flags;
    // canMessages[canMessageCount - 1].data[byteNum++] = (ubyte1)mcm->kwRequestEstimate;
    // canMessages[canMessageCount - 1].data[byteNum++] = mcm->kwRequestEstimate >> 8;
    // canMessages[canMessageCount - 1].length = byteNum;


    //----------------------------------------------------------------------------
    //Additional sensors
    //----------------------------------------------------------------------------

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    CanManager_send(me, CAN0_HIPRI, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, canMessageCount);  

}

void canOutput_sendDebugMessage1(CanManager *me, TorqueEncoder *tps, BrakePressureSensor *bps, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc, _DAQSensors *d1, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[me->write_messageLimit[1]]; 
    ubyte1 errorCount;
    float4 tempPedalPercent;   //Pedal percent float (a decimal between 0 and 1
    ubyte1 tps0Percent;  //Pedal percent int   (a number from 0 to 100)
    ubyte1 tps1Percent;
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;
    ubyte1 byteNum;

    //Inverter 1 FL Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x184;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv1->AMK_bInverterOn << 0) | (inv1->AMK_bDcOn << 1) | (inv1->AMK_bEnable << 2) | (inv1->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[CAN_CHANNELS] = inv1->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv1->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv1->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv1->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv1->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv1->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 2 FR Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x185;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv2->AMK_bInverterOn << 0) | (inv2->AMK_bDcOn << 1) | (inv2->AMK_bEnable << 2) | (inv2->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[CAN_CHANNELS] = inv2->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv2->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv2->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv2->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv2->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv2->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 3 RL Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x188;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv1->AMK_bInverterOn << 0) | (inv1->AMK_bDcOn << 1) | (inv1->AMK_bEnable << 2) | (inv1->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[CAN_CHANNELS] = inv1->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv1->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv1->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv1->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv1->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv1->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 4 RR Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x189;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((inv2->AMK_bInverterOn << 0) | (inv2->AMK_bDcOn << 1) | (inv2->AMK_bEnable << 2) | (inv2->AMK_bErrorReset << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[CAN_CHANNELS] = inv2->AMK_TorqueSetpoint;
    canMessages[canMessageCount - 1].data[3] = inv2->AMK_TorqueSetpoint >> 8;
    canMessages[canMessageCount - 1].data[4] = inv2->AMK_TorqueLimitPositiv;
    canMessages[canMessageCount - 1].data[5] = inv2->AMK_TorqueLimitPositiv >> 8;
    canMessages[canMessageCount - 1].data[6] = inv2->AMK_TorqueLimitNegativ;
    canMessages[canMessageCount - 1].data[7] = inv2->AMK_TorqueLimitNegativ >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //50B: AMK VCU Debug (3/4 Inverters)
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x50B;
    canMessages[canMessageCount - 1].data[byteNum++] = inv1->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = inv2->startUpStage;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].length = byteNum;

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    CanManager_send(me, CAN1_LOPRI, canMessages, canMessageCount);  //Send messages to CAN1
    //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, canMessageCount);  

}

/*
// pseudocode for standard can message creation (pointer held in struct of member who's data is being reported)
    // want to ignore lines where data = 0 (incase of combined CAN message from multiple struct. Individual bit flags should not be combined over mutiple structs, therefore, no workaround is made for this scenario.)
// canManager simply has a list of id's to update, direct references struct member's can message fo addition to FIFO queue

IO_CAN_DATA_FRAME CanManager_createMessage(CanManager* me, ubyte4 canID, ubyte1 canFrameType, ubyte1 data[8], IO_CAN_DATA_FRAME* canMessage){
    canMessage.id_format = canFrameType;
    canMessage.id = canID;

    for(ubyte1 i = 0; i < 7; ++i){
        if(data[i] != 0){    
        canMessage.data[i] = data[i];
        }
    }
    canMessage.length = 8;
}



organized linked list of can messages, if no ID dupe then new, if same id then update 

void canOutput_sendDebugMessage0(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, InstrumentCluster* ic, BatteryManagementSystem* bms, SafetyChecker* sc, _DriveInverter *inv1, _DriveInverter *inv2)
{
    IO_CAN_DATA_FRAME canMessages[me->write_messageLimit[0]];
    ubyte1 errorCount;
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;
    ubyte1 byteNum;


*/
