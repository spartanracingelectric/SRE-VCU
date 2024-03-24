//-------------------------------------------------------------------
//VCU Initialization 
//-------------------------------------------------------------------

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include "APDB.h"
#include "IO_DIO.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_UART.h"
//#include "IO_CAN.h"
//#include "IO_PWM.h"

//Our code
#include "initializations.h"
#include "sensors.h"
#include "canManager.h"
#include "AMKdrive.h"
#include "instrumentCluster.h"
#include "readyToDriveSound.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "wheelSpeeds.h"
#include "safety.h"
#include "sensorCalculations.h"
#include "cooling.h"
#include "daqSensors.h"
#include "drs.h"

////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////
#include "pid.h"
#include "torqueVectoring.h"
////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////


//Application Database, needed for TTC-Downloader
APDB appl_db =
    {
        0 /* ubyte4 versionAPDB        */
        ,
        {0} /* BL_T_DATE flashDate       */
            /* BL_T_DATE buildDate                   */
        ,
        {(ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                  ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26))},
        0 /* ubyte4 nodeType           */
        ,
        0 /* ubyte4 startAddress       */
        ,
        0 /* ubyte4 codeSize           */
        ,
        0 /* ubyte4 legacyAppCRC       */
        ,
        0 /* ubyte4 appCRC             */
        ,
        1 /* ubyte1 nodeNr             */
        ,
        0 /* ubyte4 CRCInit            */
        ,
        0 /* ubyte4 flags              */
        ,
        0 /* ubyte4 hook1              */
        ,
        0 /* ubyte4 hook2              */
        ,
        0 /* ubyte4 hook3              */
        ,
        APPL_START /* ubyte4 mainAddress        */
        ,
        {0, 1} /* BL_T_CAN_ID canDownloadID */
        ,
        {0, 2} /* BL_T_CAN_ID canUploadID   */
        ,
        0 /* ubyte4 legacyHeaderCRC    */
        ,
        0 /* ubyte4 version            */
        ,
        500 /* ubyte2 canBaudrate        */
        ,
        0 /* ubyte1 canChannel         */
        ,
        {0} /* ubyte1 reserved[8*4]      */
        ,
        0 /* ubyte4 headerCRC          */
};

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
extern Sensor Sensor_BPS1;
extern Sensor Sensor_SAS;
extern Sensor Sensor_TCSKnob;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_TEMP_BrakingSwitch;
extern Sensor Sensor_EcoButton;

extern Sensor Sensor_DRSButton;

/*****************************************************************************
* Main!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
    ubyte4 timestamp_startTime = 0;
    ubyte4 timestamp_EcoButton = 0;
    ubyte1 calibrationErrors; //NOT USED

    /*******************************************/
    /*        Low Level Initializations        */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems

    //Initialize serial first so we can use it to debug init of other subsystems
    //SerialManager *serialMan = SerialManager_new();
    IO_RTC_StartTime(&timestamp_startTime);

    /*******************************************/
    /*      System Level Initializations       */
    /*******************************************/

    //----------------------------------------------------------------------------
    // Check if we're on the bench or not
    //----------------------------------------------------------------------------
    bool bench;
    IO_DI_Init(IO_DI_06, IO_DI_PD_10K);
    IO_RTC_StartTime(&timestamp_startTime);
    while (IO_RTC_GetTimeUS(timestamp_startTime) < 55555)
    {
        IO_Driver_TaskBegin();

        //IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_06, &bench);

        IO_Driver_TaskEnd();
        //TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
        while (IO_RTC_GetTimeUS(timestamp_startTime) < 10000)
            ; // wait until 10ms have passed
    }
    IO_DI_DeInit(IO_DI_06);
    //SerialManager_send(serialMan, bench == TRUE ? "VCU is in bench mode.\n" : "VCU is NOT in bench mode.\n");

    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    //SerialManager_send(serialMan, "VCU objects/subsystems initializing.\n");
    vcu_initializeADC(bench); //Configure and activate all I/O pins on the VCU
    //vcu_initializeCAN();
    //vcu_initializeMCU();

    //Do some loops until the ADC stops outputting garbage values
    vcu_ADCWasteLoop();

    //vcu_init functions may have to be performed BEFORE creating CAN Manager object

    // Total CAN0 and CAN1 read & write 128 
    CanManager *canMan = CanManager_new(500, 40, 40, 500, 20, 20, 200000); //3rd param = messages per node (can0/can1; read/write)
    //can0_busSpeed ---------------------^    ^   ^   ^    ^   ^     ^         
    //can0_read_messageLimit -----------------|   |   |    |   |     |         
    //can0_write_messageLimit---------------------+   |    |   |     |         
    //can1_busSpeed-----------------------------------+    |   |     |         
    //can1_read_messageLimit-------------------------------+   |     |         
    //can1_write_messageLimit----------------------------------+     |         
    //defaultSendDelayus (Not being used currently) -----------------+         

    //----------------------------------------------------------------------------
    // Object representations of external devices
    // Most default values for things should be specified here
    //----------------------------------------------------------------------------

    //0 is for MANUAL DRS and 1 is for AUTO DRS
    ubyte1 pot_DRS_LC = 0; 

    //0 is for AWD, 1 is for RWD
    ubyte1 AMK_Mode = 0;

    ReadyToDriveSound *rtds = RTDS_new();
    //BatteryManagementSystem* bms = BMS_new();

    _DriveInverter *invFL = AmkDriver_new(FRONT_LEFT);
    _DriveInverter *invFR = AmkDriver_new(FRONT_RIGHT);
    _DriveInverter *invRL = AmkDriver_new(REAR_LEFT);
    _DriveInverter *invRR = AmkDriver_new(REAR_RIGHT);
    InstrumentCluster *ic0 = InstrumentCluster_new(0x702);
    TorqueEncoder *tps = TorqueEncoder_new(bench);
    BrakePressureSensor *bps = BrakePressureSensor_new();
    SafetyChecker *sc = SafetyChecker_new(320, 32); //Must match amp limits
    BatteryManagementSystem *bms = BMS_new(BMS_BASE_ADDRESS);
    CoolingSystem *cs = CoolingSystem_new();
    DRS *drs = DRS_new();
    _DAQSensors *d1 = DAQ_Sensor_new();
    
    ////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////
    PIDController *yawP = PID_new(parameters);
    PIDController *slipP = PID_new(parameters);
    TorqueVectoring *tv = TorqueV_new(parameters);
    ////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////

    ubyte2 tps0_calibMin = 200;  //me->tps0->sensorValue;
    ubyte2 tps0_calibMax = 1900; //me->tps0->sensorValue;
    ubyte2 tps1_calibMin = 3000; //me->tps1->sensorValue;
    ubyte2 tps1_calibMax = 4800; //me->tps1->sensorValue;

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a defined cycle time (here: 5 ms) */
    ubyte4 timestamp_mainLoopStart = 0;
    //IO_RTC_StartTime(&timestamp_calibStart);
    //SerialManager_send(serialMan, "VCU initializations complete.  Entering main loop.\n");
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_mainLoopStart);
        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        //SerialManager_send(serialMan, "VCU has entered main loop.");

        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
        //----------------------------------------------------------------------------
        // Handle data input streams
        //----------------------------------------------------------------------------
        //Get readings from our sensors and other local devices (buttons, 12v battery, etc)
        sensors_updateSensors();

        //Pull messages from CAN FIFO and update our object representations.
        //IMU DAQ will be sending to CAN1 so CAN0 is written incase there is necessity for it to be on that bus
        CanManager_read(canMan, CAN0_HIPRI, ic0, bms, sc, d1, invFL, invFR);
        CanManager_read(canMan, CAN1_LOPRI, ic0, bms, sc, d1, invRL, invRR);

        /*******************************************/
        /*          Perform Calculations           */
        /*******************************************/
        //calculations - Now that we have local sensor data and external data from CAN, we can
        //do actual processing work, from pedal travel calcs to traction control
        //calculations_calculateStuff();

        //No regen below 5kph
        sbyte2 groundSpeedKPH = 0; //SRE-7 Update
        if (groundSpeedKPH < 15)
        {
            //Regen OFF
        } else {
            if(BMS_getPackVoltage(bms) >= 38500 * 10){
                //Set regen mode to coasting
            } else {
                //Set regen mode to brakes and (coasting)
            }
        }

        //SensorValue TRUE and FALSE are reversed due to Pull Up Resistor

        //Run calibration if commanded
        //if (IO_RTC_GetTimeUS(timestamp_calibStart) < (ubyte4)5000000)
        if (Sensor_EcoButton.sensorValue == FALSE)
        {
            if (timestamp_EcoButton == 0)
            {
                //SerialManager_send(serialMan, "Eco button detected\n");
                IO_RTC_StartTime(&timestamp_EcoButton);
            }
            else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 3000000)
            {
                //SerialManager_send(serialMan, "Eco button held 3s - starting calibrations\n");
                //calibrateTPS(TRUE, 5);
                TorqueEncoder_startCalibration(tps, 5);
                BrakePressureSensor_startCalibration(bps, 5);
                Light_set(Light_dashEco, 1);
                //DIGITAL OUTPUT 4 for STATUS LED
            }
        }
        else
        {
            if (IO_RTC_GetTimeUS(timestamp_EcoButton) > 10000 && IO_RTC_GetTimeUS(timestamp_EcoButton) < 1000000)
            {
                //SerialManager_send(serialMan, "Eco mode requested\n");
            }
            timestamp_EcoButton = 0;
        }
        TorqueEncoder_update(tps);
        //Every cycle: if the calibration was started and hasn't finished, check the values again
        TorqueEncoder_calibrationCycle(tps, &calibrationErrors); //Todo: deal with calibration errors
        BrakePressureSensor_update(bps, bench);
        BrakePressureSensor_calibrationCycle(bps, &calibrationErrors);

        //DRS
        DRS_update(drs, tps, bps, pot_DRS_LC);

        CoolingSystem_calculations(cs, invFL->AMK_TempInverter, invFR->AMK_TempMotor, BMS_getHighestCellTemp_degC(bms), &Sensor_HVILTerminationSense); // SRE-7 Update: Needs to be updated in future to cool based on inverters. 
        //CoolingSystem_calculations(cs, 20, 20, 20);
        CoolingSystem_enactCooling(cs); //This belongs under outputs but it doesn't really matter for cooling

        ////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////
        pid_updateDT(yawP, timestamp_mainLoopStart/1000); //This can stay constant at 10ms don't need each iteration. But what if it changes within each section? Need to measure
        pid_calculate(yawP, invFL, invFR, invRL, invRR); //the PID output will be stored within yawP
        pid_updateDT(slipP, timestamp_mainLoopStart/1000);
        pid_calculate(slipP, invFL, invFR, invRL, invRR) //the PID output will be stored within slipP
        TV_calculateCommands(tv, invFL, invFR, invRL, invRR, yawP, slipP, + all other needed parameters); //This will do ALL the calculations for TV including yaw,slip,etc,lookuptables and place the new torque request values inside tv struct -- make sure you have all needed sensors passed in for access. You will need to add in the lookuptable object in here as well
        ////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////
        DI_calculateCommands(invFL, tps, bps, tv); //tv struct will be passed for the actual request of torque in the AMK motorcontroller
        DI_calculateCommands(invFR, tps, bps, tv);
        DI_calculateCommands(invRL, tps, bps, tv);
        DI_calculateCommands(invRR, tps, bps, tv);
        ////////////////////////////////////////////////////////////SRE-7////////////////////////////////////////////////////////////////////////////////

        SafetyChecker_update(sc, bms, tps, bps, &Sensor_HVILTerminationSense, &Sensor_LVBattery);

        /*******************************************/
        /*  Output Adjustments by Safety Checker   */
        /*******************************************/
        //Make sure to change for temp values etc
        SafetyChecker_reduceTorque(sc, bms, invFL, invFR, invRL, invRR);

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
        //MOVE INTO SAFETYCHECKER
        //SafetyChecker_setErrorLight(sc);
        Light_set(Light_dashError, (SafetyChecker_getFaults(sc) == 0) ? 0 : 1);
        //Handle motor controller startup procedures
        //MCM_relayControl(mcm0, &Sensor_HVILTerminationSense);

        //MCM_inverterControl(mcm0, tps, bps, rtds);

        DI_calculateInverterControl(invFL, &Sensor_HVILTerminationSense, tps, bps, rtds);
        DI_calculateInverterControl(invFR, &Sensor_HVILTerminationSense, tps, bps, rtds);
        DI_calculateInverterControl(invRL, &Sensor_HVILTerminationSense, tps, bps, rtds);
        DI_calculateInverterControl(invRR, &Sensor_HVILTerminationSense, tps, bps, rtds);

        IO_ErrorType err = 0;
        //Comment out to disable shutdown board control
        err = BMS_relayControl(bms);

        //Send debug data
        canOutput_sendDebugMessage0(canMan, tps, bps, ic0, bms, sc, drs, invFL, invFR);
        canOutput_sendDebugMessage1(canMan, tps, bps, ic0, bms, sc, d1, invRL, invRR, tv, yawP, slipP);

        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------
        RTDS_shutdownHelper(rtds); //Stops the RTDS from playing if the set time has elapsed

        //Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();
        //wait until the cycle time is over
        while (IO_RTC_GetTimeUS(timestamp_mainLoopStart) < 10000) // 1000 = 1ms 
        {
            IO_UART_Task(); //The task function shall be called every SW cycle.
        }

    } //end of main loop

}