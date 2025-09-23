/*****************************************************************************
 * pedalSensors.h - Unified Pedal Sensor System
 * Initial Author: Refactored from torqueEncoder.h and brakePressureSensor.h
 ******************************************************************************
 * Unified system for TPS (Torque Position Sensor) and BPS (Brake Pressure Sensor)
 * Consolidates similar functionality while maintaining explicit control paths
 ****************************************************************************/

#ifndef _PEDALSENSORS_H
#define _PEDALSENSORS_H

#include "IO_Driver.h"
#include "sensors.h"

// Pedal sensor types
typedef enum {
    PEDAL_TYPE_TPS = 0,
    PEDAL_TYPE_BPS = 1
} PedalType;

// Unified pedal sensor structure
typedef struct _PedalSensor {
    PedalType type;
    
    // Primary sensor (TPS0 or BPS0)
    Sensor *primary;
    // Secondary sensor (TPS1 for redundancy, NULL for BPS)
    Sensor *secondary;
    
    // Polarity flags
    bool primary_reverse;
    bool secondary_reverse;
    
    // Calibration state
    bool runCalibration;
    bool calibrated;
    
    // Output values
    float4 percent;           // 0.0 to 1.0 travel percentage
    bool brakesAreOn;         // TRUE if brake pedal pressed (BPS only)
    
    // TPS-specific fields
    float4 outputCurveExponent; // Pedal curve exponent (TPS only)
    
    // Calibration data (stored in EEPROM)
    ubyte4 calibMin_primary;
    ubyte4 calibMax_primary;
    ubyte4 calibMin_secondary;
    ubyte4 calibMax_secondary;
    
    // Raw sensor values
    ubyte4 primary_value;
    ubyte4 secondary_value;
    
    // Calibration timestamps
    ubyte4 timestamp_calibrationStart;
    ubyte4 timestamp_calibrationEnd;
    
    // Bench mode flag
    bool bench;
} PedalSensor;

/*****************************************************************************
* Constructor Functions
*****************************************************************************/
PedalSensor* PedalSensor_new_TPS(bool benchMode);
PedalSensor* PedalSensor_new_BPS(void);

/*****************************************************************************
* Core Functions
*****************************************************************************/
void PedalSensor_update(PedalSensor* me, bool benchMode);
void PedalSensor_calibrationCycle(PedalSensor* me, ubyte1* calibrationErrors);
void PedalSensor_startCalibration(PedalSensor* me, ubyte1 seconds);
void PedalSensor_resetCalibration(PedalSensor* me);

/*****************************************************************************
* Getter Functions
*****************************************************************************/
float4 PedalSensor_getPercent(PedalSensor* me);
bool PedalSensor_getBrakesOn(PedalSensor* me);
bool PedalSensor_getCalibrated(PedalSensor* me);

// Additional getter functions for CAN messages and compatibility
float4 PedalSensor_getIndividualSensorPercent(PedalSensor* me, ubyte1 sensorNumber);
float4 PedalSensor_getTravelPercent(PedalSensor* me);
ubyte4 PedalSensor_getCalibMinPrimary(PedalSensor* me);
ubyte4 PedalSensor_getCalibMaxPrimary(PedalSensor* me);
ubyte4 PedalSensor_getCalibMinSecondary(PedalSensor* me);
ubyte4 PedalSensor_getCalibMaxSecondary(PedalSensor* me);
ubyte4 PedalSensor_getSensorValuePrimary(PedalSensor* me);
ubyte4 PedalSensor_getSensorValueSecondary(PedalSensor* me);

/*****************************************************************************
* Safety and Validation Functions
*****************************************************************************/
bool PedalSensor_checkImplausibility(PedalSensor* me);
bool PedalSensor_checkRange(PedalSensor* me);

/*****************************************************************************
* Legacy Compatibility Functions
*****************************************************************************/
// These maintain compatibility with existing code that expects separate TPS/BPS objects
typedef PedalSensor TorqueEncoder;
typedef PedalSensor BrakePressureSensor;

#define TorqueEncoder_new(bench) PedalSensor_new_TPS(bench)
#define BrakePressureSensor_new() PedalSensor_new_BPS()

#define TorqueEncoder_update(me, bench) PedalSensor_update(me, bench)
#define BrakePressureSensor_update(me, bench) PedalSensor_update(me, bench)

#define TorqueEncoder_calibrationCycle(me, err) PedalSensor_calibrationCycle(me, err)
#define BrakePressureSensor_calibrationCycle(me, err) PedalSensor_calibrationCycle(me, err)

#define TorqueEncoder_startCalibration(me, sec) PedalSensor_startCalibration(me, sec)
#define BrakePressureSensor_startCalibration(me, sec) PedalSensor_startCalibration(me, sec)

#define TorqueEncoder_getPercent(me) PedalSensor_getPercent(me)
#define BrakePressureSensor_getPercent(me) PedalSensor_getPercent(me)

#define TorqueEncoder_getCalibrated(me) PedalSensor_getCalibrated(me)
#define BrakePressureSensor_getCalibrated(me) PedalSensor_getCalibrated(me)

#define BrakePressureSensor_getBrakesOn(me) PedalSensor_getBrakesOn(me)

// Additional legacy compatibility functions
#define TorqueEncoder_getIndividualSensorPercent(me, sensorNum) PedalSensor_getIndividualSensorPercent(me, sensorNum)
#define TorqueEncoder_getPedalTravel(me, errorCount, percent) (*percent = PedalSensor_getTravelPercent(me))
#define BrakePressureSensor_getPedalTravel(me, errorCount, percent) (*percent = PedalSensor_getTravelPercent(me))

#endif // _PEDALSENSORS_H
