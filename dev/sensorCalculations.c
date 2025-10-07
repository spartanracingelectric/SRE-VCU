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
* 2015-11-16 - Rusty Pedrosa - 
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
* FR = Pin150 = Analog Input 4
* FL = Pin138 = Analog Input 5
* RR = Pin149 = Analog Input 6
* RL = Pin137 = Analog Input 7
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

sbyte4 steering_degrees(){
    sbyte4 min_voltage = 960;
    sbyte4 max_voltage = 2560;
    sbyte4 min_angle = -90;
    sbyte4 max_angle = 90;

    sbyte4 voltage_range = max_voltage - min_voltage;
    sbyte4 angle_range = max_angle - min_angle;
    sbyte4 voltage = Sensor_SAS.sensorValue;

    sbyte4 deg = min_angle + (angle_range * (voltage - min_voltage)) / voltage_range;
    return deg;
}
