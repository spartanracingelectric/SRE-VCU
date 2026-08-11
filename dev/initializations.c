//----------------------------------------------------------------------------
// VCU Subsystem Initializations
//----------------------------------------------------------------------------
// This is where we turn on the VCU's internal power supplies and sensors.
//
// The supplies/sensors and their parameters are defined in IO_ADC.h.
// Each sensor/ADC channel/etc has different parameters, so make sure to
// check the .h files, the examples, and the documentation!
//----------------------------------------------------------------------------

//Object (sensors, controllers, etc) instantiations
//ONLY THIS FILE should have "true" version of object variables
//Everything else should have "extern" declarations of variables

#include "IO_Driver.h" //Includes datatypes, constants, etc - probably should be included in every c file
#include "IO_ADC.h"
#include "IO_PWM.h"
#include "IO_CAN.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "initializations.h"

/*****************************************************************************
* ADC
****************************************************************************/
//Turns on the VCU's ADC channels and power supplies.
void vcu_initializeADC(bool benchMode)
{
    //----------------------------------------------------------------------------
    //Power supplies/outputs
    //----------------------------------------------------------------------------
    //Analog sensor supplies
    Sensor_TPS1.ioErr_powerSet = IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);  // Pin 136 -> TPS_LO
    Sensor_TPS0.ioErr_powerSet = IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_2, IO_POWER_ON);  // Pin 147 -> TPS_HI

    //Variable power supply
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);    //IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

    //Digital/power outputs ---------------------------------------------------
    //Relay power outputs
    IO_DO_Init(IO_DO_00);    IO_DO_Set(IO_DO_00, FALSE); //mcm0 Relay
    IO_DO_Init(IO_DO_12);    IO_DO_Set(IO_DO_12, FALSE); //VCU-BMS Shutdown Relay
    IO_DO_Init(IO_DO_02);    IO_DO_Set(IO_DO_02, FALSE); //P143 - NO WIRE on SRE-7b. Pump enable is P131 = IO_DO_03.
    IO_DO_Init(IO_DO_03);    IO_DO_Set(IO_DO_03, FALSE); //P131 = PUMP EN SIG -> PDU_A.16 (NOT the fan relay)
    IO_DO_Init(IO_DO_04);    IO_DO_Set(IO_DO_04, FALSE); //P142 = FAN EN SIG -> PDU_A.39
    IO_DO_Init(IO_DO_05);    IO_DO_Set(IO_DO_05, benchMode); //power output for switches - only used on bench
    IO_DO_Init(IO_DO_06);    IO_DO_Set(IO_DO_06, FALSE); //DRS Open  - P141, NOT WIRED on SRE-7b
    IO_DO_Init(IO_DO_07);    IO_DO_Set(IO_DO_07, FALSE); //DRS Close - P129, NOT WIRED on SRE-7b

    //Lowside outputs (connects to ground when on)
    IO_DO_Init(IO_ADC_CUR_00);    IO_DO_Set(IO_ADC_CUR_00, FALSE); //Brake light
    IO_DO_Init(IO_ADC_CUR_01);    IO_DO_Set(IO_ADC_CUR_01, FALSE); //P108 = BE2 LT SIG -> DASH.8 (RTD ring light)
    IO_DO_Init(IO_ADC_CUR_02);    IO_DO_Set(IO_ADC_CUR_02, FALSE); //Err - P119, NOT WIRED
    IO_DO_Init(IO_ADC_CUR_03);    IO_DO_Set(IO_ADC_CUR_03, FALSE); //RTD - P107, NOT WIRED (real RTD light is CUR_01)
    //Wheel Speed Sensor supplies
    //Sensor_WSS_FL.ioErr_powerInit = Sensor_WSS_FR.ioErr_powerInit = Sensor_WSS_RL.ioErr_powerInit = Sensor_WSS_RR.ioErr_powerInit = IO_DO_Init(IO_DO_07); // WSS power
    // IO_POWER_Set (IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);

    //Digital PWM outputs ---------------------------------------------------
    // RTD Sound (dash alertor)
    IO_PWM_Init(IO_PWM_03, 750, TRUE, FALSE, 0, FALSE, NULL);   //P105 = BE2 ALRT SIG -> DASH.6
    IO_PWM_SetDuty(IO_PWM_03, 0, NULL);
    //IO_PWM_01 = P106, NOT WIRED on SRE-7b
    
    // Rad Fans (SR-14 and above) - P117 NOT WIRED on SRE-7b, and the PDU's PWM fan input (PDU_B.35/B.36) was never built.
    // Fans are on/off via IO_DO_04 (P142 = FAN EN SIG -> PDU_A.39) in Light_set() instead.
    //IO_PWM_Init(IO_PWM_02, 100, TRUE, FALSE, 0, FALSE, NULL); //Pin, Frequency Hz, Boolean for pos polarity, current measurement enabled bool, weird other pin (current), no diagram margin, not safety critical
    //IO_PWM_SetDuty(IO_PWM_02, .90 * 0xFFFF, NULL); //Pin, 0 - 65535, Feedback Measurement

    //Accum fan signal
    IO_PWM_Init(IO_PWM_04, 100, TRUE, FALSE, 0, FALSE, NULL);
    IO_PWM_SetDuty(IO_PWM_04, 1.00 * 0xFFFF, NULL);    //Default 100%, build up momentum in fans or whatever? Lol

    //----------------------------------------------------------------------------
    //ADC channels
    //----------------------------------------------------------------------------
    //TPS+BPS
    extern Sensor Sensor_BenchTPS0; //wtf where are these even defined?
    extern Sensor Sensor_BenchTPS1;

    //IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    //IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);

    //TPS/BPS
    //Sensor_BPS0.ioErr_init = IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    if (benchMode == TRUE)
    {
        //Redo BPS ratiometric
        Sensor_TPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RESISTIVE, 0, 0, 0, NULL);  //P140 TPS HI SIG
        Sensor_TPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_06, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
        Sensor_BPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_07, IO_ADC_RESISTIVE, 0, 0, 0, NULL);  //P137 BPS F SIG
        Sensor_BPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    }
    else //Not bench mode
    {
        //In the future, production TPS will be digital instead of analog (see PWD section, below)
        //Sensor_TPS0.ioErr_signalInit = IO_PWD_PulseInit(IO_PWM_00, IO_PWD_HIGH_TIME);
        //Sensor_TPS1.ioErr_signalInit = IO_PWD_PulseInit(IO_PWM_01, IO_PWD_HIGH_TIME);
        //Redo BPS ratiometric
        Sensor_TPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_2, NULL);  //P140 TPS HI SIG
        Sensor_TPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_06, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
        Sensor_BPS0.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_07, IO_ADC_ABSOLUTE,    0, 0, 0, NULL);
        Sensor_BPS1.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_ABSOLUTE,    0, 0, 0, NULL);
    }

    //Unused
    //IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

    //SAS (Steering Angle Sensor)
    Sensor_SAS.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_ABSOLUTE, 0, 0, 0, NULL);  //P150 SAS SIG

    //DRS
    Sensor_WPS_FL.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_VAR_07, IO_ADC_ABSOLUTE, IO_ADC_RANGE_30V, 0, 0, NULL);
    Sensor_WPS_FR.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_VAR_05, IO_ADC_ABSOLUTE, IO_ADC_RANGE_30V, 0, 0, NULL);
    Sensor_WPS_RL.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_VAR_03, IO_ADC_ABSOLUTE, IO_ADC_RANGE_30V, 0, 0, NULL);
    Sensor_WPS_RR.ioErr_signalInit = IO_ADC_ChannelInit(IO_ADC_VAR_01, IO_ADC_ABSOLUTE, IO_ADC_RANGE_30V, 0, 0, NULL);
    
    //TCS Pot
    //IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL);

    //Unused
    //IO_ADC_ChannelInit(IO_ADC_5V_05, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    //IO_ADC_ChannelInit(IO_ADC_5V_06, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    //IO_ADC_ChannelInit(IO_ADC_5V_07, IO_ADC_RESISTIVE, 0, 0, 0, NULL);

    //----------------------------------------------------------------------------
    //PWD channels
    //----------------------------------------------------------------------------
    //TPS
    //MOVED TO TPS/BPS BLOCK ABOVE

    //Wheel Speed Sensors (Pulse Width Detection)

    //IO_RTC_StartTime(&Sensor_WSS_FL.timestamp);
    //IO_RTC_StartTime(&Sensor_WSS_FR.timestamp);
    //IO_RTC_StartTime(&Sensor_WSS_RL.timestamp);
    //IO_RTC_StartTime(&Sensor_WSS_RR.timestamp);

    //Sensor_WSS_FL.heldSensorValue = Sensor_WSS_FR.heldSensorValue = Sensor_WSS_RL.heldSensorValue = Sensor_WSS_RR.heldSensorValue = 0;

    //Sensor_WSS_FL.ioErr_signalInit = IO_PWD_ComplexInit(IO_PWD_10, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4, IO_PWD_THRESH_1_25V, NULL, NULL); //P274
    //Sensor_WSS_FR.ioErr_signalInit = IO_PWD_ComplexInit(IO_PWD_08, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4, IO_PWD_THRESH_1_25V, NULL, NULL); //P275
    //Sensor_WSS_RL.ioErr_signalInit = IO_PWD_ComplexInit(IO_PWD_09, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4, IO_PWD_THRESH_1_25V, NULL, NULL); //P268
    //Sensor_WSS_RR.ioErr_signalInit = IO_PWD_ComplexInit(IO_PWD_11, IO_PWD_LOW_TIME, IO_PWD_FALLING_VAR, IO_PWD_RESOLUTION_0_8, 4, IO_PWD_THRESH_1_25V, NULL, NULL); //P267
    //Maybe look for falling edge because we're using NPN/sinking WSS? 

    //----------------------------------------------------------------------------
    //Switches
    //----------------------------------------------------------------------------
    Sensor_RTDButton.ioErr_signalInit = IO_DI_Init(IO_DI_04, IO_DI_PD_10K);     //P261 = BE2 SIG-VCU -> SWC lower-right (simulating RTD). Sourcing switch, so pull DOWN.
     Sensor_EcoButton.ioErr_signalInit = IO_DI_Init(IO_DI_01, IO_DI_PU_10K);     //P256 = CAL BTN SIG -> DASH.7 (calibration button)
    //Sensor_TCSSwitchUp.ioErr_signalInit = IO_DI_Init(IO_DI_02, IO_DI_PU_10K);   //TCS Switch A
    Sensor_TVButton.ioErr_signalInit = IO_DI_Init(IO_DI_03, IO_DI_PU_10K); // Torque Control Enable Button
    Sensor_HVILTerminationSense.ioErr_signalInit = IO_DI_Init(IO_DI_07, IO_DI_PD_10K); //P253 [TERM VCU], high = HV present. Flagswitch presents voltage when closed, floats when open, so pull DOWN.

    //TV and DRS Button may be switched on SRE-7

    // Sensor_IO_DI_06.ioErr_signalInit = IO_DI_Init(IO_DI_06, IO_DI_PD_10K); //Unused - P260, HVIL term sense is DI_07 (above)
}

//----------------------------------------------------------------------------
// Waste CPU cycles until we have valid data
//----------------------------------------------------------------------------
void vcu_ADCWasteLoop(void)
{
    bool tempFresh = FALSE;
    ubyte2 tempData;
    ubyte4 timestamp_sensorpoll = 0;
    IO_RTC_StartTime(&timestamp_sensorpoll);
    while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 1000000)
    {
        IO_Driver_TaskBegin();

        IO_PWM_SetDuty(IO_PWM_03, 0, NULL);

        IO_DO_Set(IO_DO_00, FALSE); //False = low
        IO_DO_Set(IO_DO_12, FALSE); //HVIL shutdown relay

        //IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_04, &tempData);
        IO_DI_Get(IO_DI_05, &tempData);
        IO_ADC_Get(IO_ADC_5V_00, &tempData, &tempFresh);
        IO_ADC_Get(IO_ADC_5V_01, &tempData, &tempFresh);

        IO_Driver_TaskEnd();
        //TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 12500)
            ; // wait until 1/8/10s (125ms) have passed
    }
}

/*****************************************************************************
* Sensors
****************************************************************************/
Sensor Sensor_TPS0; // = { 0, 0.5, 4.5 };
Sensor Sensor_TPS1; // = { 0, 4.5, 0.5 };
Sensor Sensor_WPS_FL;
Sensor Sensor_WPS_FR;
Sensor Sensor_WPS_RL;
Sensor Sensor_WPS_RR;
Sensor Sensor_BPS0; // = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
Sensor Sensor_BPS1;  // = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)
Sensor Sensor_SAS;    // = { 4 };
Sensor Sensor_LVBattery;

Sensor Sensor_TCSKnob;

Sensor Sensor_RTDButton;
Sensor Sensor_EcoButton;
Sensor Sensor_HVILTerminationSense;
Sensor Sensor_TVButton;

Sensor Sensor_DRSButton;
Sensor Sensor_DRSKnob;

//Switches
//precharge failure

//Other
extern Sensor Sensor_LVBattery; // = { 0xA };  //Note: There will be no init for this "sensor"
