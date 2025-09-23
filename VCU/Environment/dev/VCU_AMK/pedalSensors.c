/*****************************************************************************
 * pedalSensors.c - Unified Pedal Sensor System
 * Initial Author: Refactored from torqueEncoder.c and brakePressureSensor.c
 ******************************************************************************
 * Unified implementation for TPS (Torque Position Sensor) and BPS (Brake Pressure Sensor)
 * Consolidates similar functionality while maintaining explicit control paths
 ****************************************************************************/

#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "pedalSensors.h"
#include "sensors.h"

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
extern Sensor Sensor_BPS1;
extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

/*****************************************************************************
* Constructor Functions
*****************************************************************************/

PedalSensor* PedalSensor_new_TPS(bool benchMode)
{
    PedalSensor* me = (PedalSensor*)malloc(sizeof(PedalSensor));
    me->type = PEDAL_TYPE_TPS;
    me->bench = benchMode;
    
    // Set up primary and secondary sensors
    me->primary = (benchMode == TRUE) ? &Sensor_BenchTPS0 : &Sensor_TPS0;
    me->secondary = (benchMode == TRUE) ? &Sensor_BenchTPS1 : &Sensor_TPS1;
    
    // TPS polarity settings
    me->primary_reverse = FALSE;
    me->secondary_reverse = TRUE;
    
    // TPS-specific settings
    me->outputCurveExponent = 1.0; // Linear by default
    
    // Initialize calibration data
    me->calibMin_primary = 0;
    me->calibMax_primary = 0;
    me->calibMin_secondary = 0;
    me->calibMax_secondary = 0;
    
    // Initialize state
    me->percent = 0;
    me->brakesAreOn = FALSE; // Not applicable for TPS
    me->runCalibration = FALSE;
    me->calibrated = FALSE;
    me->primary_value = 0;
    me->secondary_value = 0;
    me->timestamp_calibrationStart = 0;
    me->timestamp_calibrationEnd = 0;
    
    return me;
}

PedalSensor* PedalSensor_new_BPS(void)
{
    PedalSensor* me = (PedalSensor*)malloc(sizeof(PedalSensor));
    me->type = PEDAL_TYPE_BPS;
    me->bench = FALSE; // BPS doesn't have bench mode
    
    // Set up primary sensor only (BPS typically has one sensor)
    me->primary = &Sensor_BPS0;
    me->secondary = NULL; // BPS doesn't have secondary sensor
    
    // BPS polarity settings
    me->primary_reverse = FALSE;
    me->secondary_reverse = FALSE;
    
    // Initialize calibration data
    me->calibMin_primary = 0;
    me->calibMax_primary = 0;
    me->calibMin_secondary = 0;
    me->calibMax_secondary = 0;
    
    // Initialize state
    me->percent = 0;
    me->brakesAreOn = FALSE;
    me->runCalibration = FALSE;
    me->calibrated = FALSE;
    me->primary_value = 0;
    me->secondary_value = 0;
    me->timestamp_calibrationStart = 0;
    me->timestamp_calibrationEnd = 0;
    
    return me;
}

/*****************************************************************************
* Core Functions
*****************************************************************************/

void PedalSensor_update(PedalSensor* me, bool benchMode)
{
    if (me->type == PEDAL_TYPE_TPS) {
        // TPS update logic
        me->primary_value = me->primary->sensorValue;
        me->secondary_value = me->secondary->sensorValue;
        
        // Calculate individual sensor percentages
        float4 tps0_percent = 0;
        float4 tps1_percent = 0;
        
        if (me->calibrated) {
            // Primary sensor (TPS0)
            ubyte4 rawValue = me->primary_value;
            if (me->primary_reverse) {
                rawValue = me->calibMax_primary - rawValue;
            }
            tps0_percent = getPercent(rawValue, me->calibMin_primary, me->calibMax_primary, TRUE);
            
            // Secondary sensor (TPS1)
            rawValue = me->secondary_value;
            if (me->secondary_reverse) {
                rawValue = me->calibMax_secondary - rawValue;
            }
            tps1_percent = getPercent(rawValue, me->calibMin_secondary, me->calibMax_secondary, TRUE);
        }
        
        // Average the two sensors for final output
        me->percent = (tps0_percent + tps1_percent) / 2.0;
        
        // Apply output curve (TPS only)
        if (me->percent > 0) {
            me->percent = powf(me->percent, me->outputCurveExponent);
        }
        
    } else if (me->type == PEDAL_TYPE_BPS) {
        // BPS update logic
        me->primary_value = me->primary->sensorValue;
        
        if (me->calibrated) {
            ubyte4 rawValue = me->primary_value;
            if (me->primary_reverse) {
                rawValue = me->calibMax_primary - rawValue;
            }
            me->percent = getPercent(rawValue, me->calibMin_primary, me->calibMax_primary, TRUE);
        }
        
        // Set brake status (BPS only)
        me->brakesAreOn = (me->percent > 0.1); // 10% threshold
    }
}

void PedalSensor_calibrationCycle(PedalSensor* me, ubyte1* calibrationErrors)
{
    if (!me->runCalibration) {
        return;
    }
    
    ubyte4 currentTime = IO_RTC_GetTimeUS(0);
    
    if (me->type == PEDAL_TYPE_TPS) {
        // TPS calibration logic
        if (currentTime - me->timestamp_calibrationStart < 5000000) { // 5 seconds
            // Still in calibration period
            ubyte4 tps0_value = me->primary->sensorValue;
            ubyte4 tps1_value = me->secondary->sensorValue;
            
            if (tps0_value < me->calibMin_primary || me->calibMin_primary == 0) {
                me->calibMin_primary = tps0_value;
            }
            if (tps0_value > me->calibMax_primary) {
                me->calibMax_primary = tps0_value;
            }
            
            if (tps1_value < me->calibMin_secondary || me->calibMin_secondary == 0) {
                me->calibMin_secondary = tps1_value;
            }
            if (tps1_value > me->calibMax_secondary) {
                me->calibMax_secondary = tps1_value;
            }
        } else {
            // Calibration complete
            me->runCalibration = FALSE;
            me->calibrated = TRUE;
            me->timestamp_calibrationEnd = currentTime;
        }
        
    } else if (me->type == PEDAL_TYPE_BPS) {
        // BPS calibration logic
        if (currentTime - me->timestamp_calibrationStart < 3000000) { // 3 seconds
            // Still in calibration period
            ubyte4 bps0_value = me->primary->sensorValue;
            
            if (bps0_value < me->calibMin_primary || me->calibMin_primary == 0) {
                me->calibMin_primary = bps0_value;
            }
            if (bps0_value > me->calibMax_primary) {
                me->calibMax_primary = bps0_value;
            }
        } else {
            // Calibration complete
            me->runCalibration = FALSE;
            me->calibrated = TRUE;
            me->timestamp_calibrationEnd = currentTime;
        }
    }
}

void PedalSensor_startCalibration(PedalSensor* me, ubyte1 seconds)
{
    me->runCalibration = TRUE;
    me->calibrated = FALSE;
    me->timestamp_calibrationStart = IO_RTC_GetTimeUS(0);
    
    // Reset calibration values
    me->calibMin_primary = 0xFFFF;
    me->calibMax_primary = 0;
    me->calibMin_secondary = 0xFFFF;
    me->calibMax_secondary = 0;
}

void PedalSensor_resetCalibration(PedalSensor* me)
{
    me->calibrated = FALSE;
    me->runCalibration = FALSE;
    me->calibMin_primary = 0;
    me->calibMax_primary = 0;
    me->calibMin_secondary = 0;
    me->calibMax_secondary = 0;
}

/*****************************************************************************
* Getter Functions
*****************************************************************************/

float4 PedalSensor_getPercent(PedalSensor* me)
{
    return me->percent;
}

bool PedalSensor_getBrakesOn(PedalSensor* me)
{
    return me->brakesAreOn;
}

bool PedalSensor_getCalibrated(PedalSensor* me)
{
    return me->calibrated;
}

// Additional getter functions for CAN messages and compatibility
float4 PedalSensor_getIndividualSensorPercent(PedalSensor* me, ubyte1 sensorNumber)
{
    if (me->type == PEDAL_TYPE_TPS && me->calibrated) {
        if (sensorNumber == 0) {
            // Primary sensor (TPS0)
            ubyte4 rawValue = me->primary_value;
            if (me->primary_reverse) {
                rawValue = me->calibMax_primary - rawValue;
            }
            return getPercent(rawValue, me->calibMin_primary, me->calibMax_primary, TRUE);
        } else if (sensorNumber == 1) {
            // Secondary sensor (TPS1)
            ubyte4 rawValue = me->secondary_value;
            if (me->secondary_reverse) {
                rawValue = me->calibMax_secondary - rawValue;
            }
            return getPercent(rawValue, me->calibMin_secondary, me->calibMax_secondary, TRUE);
        }
    }
    return 0.0;
}

float4 PedalSensor_getTravelPercent(PedalSensor* me)
{
    return me->percent;
}

ubyte4 PedalSensor_getCalibMinPrimary(PedalSensor* me)
{
    return me->calibMin_primary;
}

ubyte4 PedalSensor_getCalibMaxPrimary(PedalSensor* me)
{
    return me->calibMax_primary;
}

ubyte4 PedalSensor_getCalibMinSecondary(PedalSensor* me)
{
    return me->calibMin_secondary;
}

ubyte4 PedalSensor_getCalibMaxSecondary(PedalSensor* me)
{
    return me->calibMax_secondary;
}

ubyte4 PedalSensor_getSensorValuePrimary(PedalSensor* me)
{
    return me->primary_value;
}

ubyte4 PedalSensor_getSensorValueSecondary(PedalSensor* me)
{
    return me->secondary_value;
}

/*****************************************************************************
* Safety and Validation Functions
*****************************************************************************/

bool PedalSensor_checkImplausibility(PedalSensor* me)
{
    if (me->type == PEDAL_TYPE_TPS && me->calibrated) {
        // Check TPS implausibility (EV2.3.5 rule)
        float4 tps0_percent = 0;
        float4 tps1_percent = 0;
        
        ubyte4 rawValue = me->primary_value;
        if (me->primary_reverse) {
            rawValue = me->calibMax_primary - rawValue;
        }
        tps0_percent = getPercent(rawValue, me->calibMin_primary, me->calibMax_primary, TRUE);
        
        rawValue = me->secondary_value;
        if (me->secondary_reverse) {
            rawValue = me->calibMax_secondary - rawValue;
        }
        tps1_percent = getPercent(rawValue, me->calibMin_secondary, me->calibMax_secondary, TRUE);
        
        // Check for implausibility (deviation > 10%)
        return (fabsf(tps1_percent - tps0_percent) > 0.1);
    }
    
    return FALSE;
}

bool PedalSensor_checkRange(PedalSensor* me)
{
    // Check if sensor values are within expected range
    if (me->primary && me->primary->sensorValue < me->primary->specMin || 
        me->primary->sensorValue > me->primary->specMax) {
        return FALSE;
    }
    
    if (me->secondary && me->secondary->sensorValue < me->secondary->specMin || 
        me->secondary->sensorValue > me->secondary->specMax) {
        return FALSE;
    }
    
    return TRUE;
}