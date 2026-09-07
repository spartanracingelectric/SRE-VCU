/*****************************************************************************
* Output Calculations
******************************************************************************
* Takes properties from devices (such as raw sensor values [ohms, voltage], 
* MCU/BMS CAN messages, etc), performs calculations with that data, and updates
* the relevant objects' properties.
*
* This includes sensor calculations, motor controller control calculations,
* traction control, BMS/safety calculations, etc.
* (May need to split this up later)
*
* For example: GetThrottlePosition() takes the raw TPS voltages from the TPS 
* sensor objects and returns the throttle pedal percent.  This function does 
* NOT update the sensor objects, but it would be acceptable for another 
* function in this file to do so.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-11-16 - Rusty Pedrosa
* 2026-09-06 - Akash Karthik
*****************************************************************************/

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "sensorCalculations.h"
#include "sensors.h"
#include "mathFunctions.h"

extern Sensor Sensor_SAS;

//Theoretical ground speed
//63360 inches per mile. Wish we could use metric.
double rpm_to_mph(double rpm) {
    return (double)((3.14159265*WHEEL_DIAMETER_D*rpm*60.0) / 63360.0);
}

/*****************************************************************************
* Shock pot(iometer) functions - FOUR NEEDED
* FL = Pin111 = IO_ADC_VAR_07 (Analog Input 7, 0..32V)
* FR = Pin112 = IO_ADC_VAR_05 (Analog Input 5, 0..32V)
* RL = Pin113 = IO_ADC_VAR_03 (Analog Input 3, 0..32V)
* RR = Pin114 = IO_ADC_VAR_01 (Analog Input 1, 0..32V)
* 0 = ride height
****************************************************************************/
//Input: Ohms
//Outputs: ???
//See VCU Manual section 5.8.8 - there are different outputs depending on sensor resistance
//Example sensor: Active Sensors CLS0950
//Resistive range: 0.4 to 6.0 kohm
//ShockPot.

/****************************************************************************
 * Steering Angle Sensor (SAS)
 * Input: Voltage
 * Output: Degrees
 * **************************************************************************/

bool steering_degrees(sbyte4 *deg_out)
{
    sbyte4 mv = (sbyte4)Sensor_SAS.sensorValue;
    sbyte4 deg;
    *deg_out = 0;
    if ((Sensor_SAS.ioErr_signalGet != IO_E_OK) || (Sensor_SAS.fresh == FALSE))
    {
        return FALSE;
    }
    if ((mv < (SAS_DIAG_LOW_MV - SAS_DIAG_MARGIN_MV)) ||
        (mv > (SAS_DIAG_HIGH_MV + SAS_DIAG_MARGIN_MV)))
    {
        return FALSE;
    }

    deg = ((mv - SAS_CENTER_MV) * 10) / SAS_MV_PER_DEG_X10;


    //if we mounted it backwards then we can just invert the sign
#if SAS_INVERT
    deg = -deg;
#endif
    if (deg >  SAS_MAX_DEG) { deg =  SAS_MAX_DEG; }
    if (deg < -SAS_MAX_DEG) { deg = -SAS_MAX_DEG; }

    *deg_out = deg;
    return TRUE;
}
