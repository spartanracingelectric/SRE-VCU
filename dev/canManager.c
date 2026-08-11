
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
void CanManager_read(CanManager *me, CanChannel channel, InstrumentCluster *ic, BatteryManagementSystem *bms, SafetyChecker *sc, _DAQSensors *d1, _Powertrain *powertrain)
{
    IO_CAN_DATA_FRAME canMessages[(channel == CAN0_HIPRI ? me->read_messageLimit[0] : me->read_messageLimit[1])];
    ubyte1 canMessageCount;  //FIFO queue only holds 128 messages max

    //Read messages from hi/lopri channel 
    *(channel == CAN0_HIPRI ? &me->ioErr_read[0] : &me->ioErr_read[1]) =
    IO_CAN_ReadFIFO((channel == CAN0_HIPRI ? me->readHandle[0] : me->readHandle[1])
                    , canMessages
                    , (channel == CAN0_HIPRI ? me->read_messageLimit[0] : me->read_messageLimit[1])
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
            //Inverter FL 1 (CAN0)
            DI_parseCanMessage(powertrain->motor[0], &canMessages[currMessage]);
            break;
        case 0x285:

            break;
        case 0x284:
            //Inverter FR 1 (CAN0)
            DI_parseCanMessage(powertrain->motor[1], &canMessages[currMessage]);
            break;
        case 0x286:

            break;
        case 0x287:
            //Inverter RL 1 (CAN1)
            DI_parseCanMessage(powertrain->motor[2], &canMessages[currMessage]);
            break;
        case 0x289:

            break;
        case 0x288:
            //Inverter RR 1 (CAN1)
            DI_parseCanMessage(powertrain->motor[3], &canMessages[currMessage]);
            break;
        case 0x290:

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
void canOutput_sendDebugMessage0(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, InstrumentCluster* ic, BatteryManagementSystem* bms, SafetyChecker* sc, _Powertrain *powertrain)
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
    canMessages[canMessageCount - 1].data[byteNum++] = Sensor_EcoButton.sensorValue;
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

    //24v LV battery (2x12V in series) - thresholds are the 12V curve doubled
    float4 LVBatterySOC = 0;
    if (Sensor_LVBattery.sensorValue < 25460)
        LVBatterySOC = .0 + .1 * getPercent(Sensor_LVBattery.sensorValue, 18400, 25460, FALSE);
    else if (Sensor_LVBattery.sensorValue < 25732)
        LVBatterySOC = .1 + .1 * getPercent(Sensor_LVBattery.sensorValue, 25460, 25732, FALSE);
    else if (Sensor_LVBattery.sensorValue < 25992)
        LVBatterySOC = .2 + .1 * getPercent(Sensor_LVBattery.sensorValue, 25732, 25992, FALSE);
    else if (Sensor_LVBattery.sensorValue < 26208)
        LVBatterySOC = .3 + .1 * getPercent(Sensor_LVBattery.sensorValue, 25992, 26208, FALSE);
    else if (Sensor_LVBattery.sensorValue < 26232)
        LVBatterySOC = .4 + .1 * getPercent(Sensor_LVBattery.sensorValue, 26208, 26232, FALSE);
    else if (Sensor_LVBattery.sensorValue < 26260)
        LVBatterySOC = .5 + .1 * getPercent(Sensor_LVBattery.sensorValue, 26232, 26260, FALSE);
    else if (Sensor_LVBattery.sensorValue < 26320)
        LVBatterySOC = .6 + .1 * getPercent(Sensor_LVBattery.sensorValue, 26260, 26320, FALSE);
    else if (Sensor_LVBattery.sensorValue < 26540)
        LVBatterySOC = .7 + .1 * getPercent(Sensor_LVBattery.sensorValue, 26320, 26540, FALSE);
    else if (Sensor_LVBattery.sensorValue < 26600)
        LVBatterySOC = .8 + .1 * getPercent(Sensor_LVBattery.sensorValue, 26540, 26600, FALSE);
    else //if (Sensor_LVBattery.sensorValue < 28680)
        LVBatterySOC = .9 + .1 * getPercent(Sensor_LVBattery.sensorValue, 26600, 28680, TRUE);  //TRUE = clamp, so SOC can't exceed 100% on charge

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
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[0]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[0]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[1]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[1]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[2]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[2]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[3]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[3]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].length = byteNum;

    //50B: AMK VCU Debug
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[0]->startUpStage | powertrain->motor[1]->startUpStage << 4 ;
    canMessages[canMessageCount - 1].data[byteNum++] = powertrain->motor[2]->startUpStage | powertrain->motor[3]->startUpStage << 4 ;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 1; //Rough Version Control. Should be updated to proper versioning later.
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

void canOutput_sendDebugMessage1(CanManager *me, _Powertrain *powertrain)
{
    IO_CAN_DATA_FRAME canMessages[me->write_messageLimit[1]]; 
    ubyte2 canMessageCount = 0;


    //Inverter 1 FL Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x184;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((powertrain->motor[0]->AMK_InverterOn_send << 0) | (powertrain->motor[0]->AMK_DcOn_send << 1) | (powertrain->motor[0]->AMK_Enable_send << 2) | (powertrain->motor[0]->AMK_ErrorReset_send << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = powertrain->motor[0]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[3] = powertrain->motor[0]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[4] = powertrain->motor[0]->AMK_TorqueLimitPositive_send;
    canMessages[canMessageCount - 1].data[5] = powertrain->motor[0]->AMK_TorqueLimitPositive_send >> 8;
    canMessages[canMessageCount - 1].data[6] = powertrain->motor[0]->AMK_TorqueLimitNegative_send;
    canMessages[canMessageCount - 1].data[7] = powertrain->motor[0]->AMK_TorqueLimitNegative_send >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 2 FR Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x185;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((powertrain->motor[1]->AMK_InverterOn_send << 0) | (powertrain->motor[1]->AMK_DcOn_send << 1) | (powertrain->motor[1]->AMK_Enable_send << 2) | (powertrain->motor[1]->AMK_ErrorReset_send << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = powertrain->motor[1]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[3] = powertrain->motor[1]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[4] = powertrain->motor[1]->AMK_TorqueLimitPositive_send;
    canMessages[canMessageCount - 1].data[5] = powertrain->motor[1]->AMK_TorqueLimitPositive_send >> 8;
    canMessages[canMessageCount - 1].data[6] = powertrain->motor[1]->AMK_TorqueLimitNegative_send;
    canMessages[canMessageCount - 1].data[7] = powertrain->motor[1]->AMK_TorqueLimitNegative_send >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 3 RL Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x188;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((powertrain->motor[2]->AMK_InverterOn_send << 0) | (powertrain->motor[2]->AMK_DcOn_send << 1) | (powertrain->motor[2]->AMK_Enable_send << 2) | (powertrain->motor[2]->AMK_ErrorReset_send << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = powertrain->motor[2]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[3] = powertrain->motor[2]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[4] = powertrain->motor[2]->AMK_TorqueLimitPositive_send;
    canMessages[canMessageCount - 1].data[5] = powertrain->motor[2]->AMK_TorqueLimitPositive_send >> 8;
    canMessages[canMessageCount - 1].data[6] = powertrain->motor[2]->AMK_TorqueLimitNegative_send;
    canMessages[canMessageCount - 1].data[7] = powertrain->motor[2]->AMK_TorqueLimitNegative_send >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //Inverter 4 RR Command Message
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x189;
    canMessages[canMessageCount - 1].data[0] = 0; //ReservedIgnore1
    canMessages[canMessageCount - 1].data[1] = (ubyte1)((powertrain->motor[3]->AMK_InverterOn_send << 0) | (powertrain->motor[3]->AMK_DcOn_send << 1) | (powertrain->motor[3]->AMK_Enable_send << 2) | (powertrain->motor[3]->AMK_ErrorReset_send << 3));
    canMessages[canMessageCount - 1].data[1] &= 0x0F;  //ReservedIgnore2
    canMessages[canMessageCount - 1].data[2] = powertrain->motor[3]->AMK_TorqueRequest_send;
    canMessages[canMessageCount - 1].data[3] = powertrain->motor[3]->AMK_TorqueRequest_send >> 8;
    canMessages[canMessageCount - 1].data[4] = powertrain->motor[3]->AMK_TorqueLimitPositive_send;
    canMessages[canMessageCount - 1].data[5] = powertrain->motor[3]->AMK_TorqueLimitPositive_send >> 8;
    canMessages[canMessageCount - 1].data[6] = powertrain->motor[3]->AMK_TorqueLimitNegative_send;
    canMessages[canMessageCount - 1].data[7] = powertrain->motor[3]->AMK_TorqueLimitNegative_send >> 8;
    canMessages[canMessageCount - 1].length = 8;

    //50B: AMK VCU Debug (3/4 Inverters)
    canMessageCount++;
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = 0x50B;
    canMessages[canMessageCount - 1].data[0] = powertrain->motor[0]->startUpStage | powertrain->motor[1]->startUpStage << 4 ;
    canMessages[canMessageCount - 1].data[1] = powertrain->motor[2]->startUpStage | powertrain->motor[3]->startUpStage << 4 ;
    canMessages[canMessageCount - 1].data[2] = 0;
    canMessages[canMessageCount - 1].data[3] = 0;
    canMessages[canMessageCount - 1].data[4] = 0;
    canMessages[canMessageCount - 1].data[5] = 0;
    canMessages[canMessageCount - 1].data[6] = 0;
    canMessages[canMessageCount - 1].data[7] = 0;
    canMessages[canMessageCount - 1].length = 8;

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    CanManager_send(me, CAN1_LOPRI, canMessages, canMessageCount);  //Send messages to CAN1
    //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, canMessageCount);  

}

/*
// pseudocode for standard can message creation (pointer held in struct of member who's data is being reported)
    // want to ignore lines where data = 0 (incase of combined CAN message from multiple struct. Individual bit flags should not be combined over mutiple structs, therefore, no workaround is made for this scenario.)
// canManager simply has a list of id's to update, direct references struct member's can message fo addition to FIFO queue

//me->lastvalid keeps track of last obtainMessageAddress() pointer
IO_CAN_DATA_FRAME * canManager_obtainMessageAddress(CanManager* me, ubyte4 canID){

    for(ubyte1 i = 0; i < me->lastvalid; ++i){
        if(me->canMessages0[i].id = canID)
        {   return &me->canMessages0[i]; }
    }
    if(me->lastvalid < me->write_messageLimit[0]){
        ++me->lastvalid;
        me->canMessages0[me->lastvalid].id = canID;
        return &me->canMessages0[me->lastvalid];
    }
    return NULL;
}

IO_CAN_DATA_FRAME canManager_fillMessage(IO_CAN_DATA_FRAME* canMessage, ubyte1 data, ubyte1 byte){

    for(ubyte1 i = 0; i < 7; ++i){
        if(canMessage.data[i] != 0){    
        canMessage.data[i] = data[i];
        }
    }
}

Could also remove the copy of the data[i], but would have to have individual copied implementations for the same process of checking if a byte is being used by someone else. Maybe come up with a way to avoid both things?
//
//
//
void canManager_buildMessageLine0(CanManager* me){
    
    // me->lastvalid <= me->write_messageLimit[0];

    for( ubyte1 i = 0; i < me->lastvalid; ++i){
        // filling out all default values before messages are sent
        me->canMessages0[i].id_format = IO_CAN_STD_FRAME;
        me->canMessages0[i].length = 8;
    }

}

//implementation for using
TPS tps_init(){
    ...
    tps->canMessage = canManager_obtainMessageAddress(can0, 0x500);
    ...
}

tps_update(TPS* tps){
    ...
    Sensor_TPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_06, &Sensor_TPS0.sensorValue, &Sensor_TPS0.fresh);
    canManager_fillMessage(tps->canMessage, tps->sensorValue0, 2);
    canManager_fillMessage(tps->canMessage, (tps->sensorValue0 >> 8), 3);
    ...
}

void main(){
    ...
    canOutput_buildMessageLine0(CanManager* me);
    canManager_send(canManager, CAN0_HIPRI, canManager->canMessages0H, canManager->lastvalid);  //Send messages to CAN0 (no more canOutput_sendDebugMessage0/1)
    ...
}
*/
