/*****************************************************************************
 * vcu_globals.c - Centralized Global Variables and Constants Implementation
 * Optimized Version
 ******************************************************************************
 * This file implements the global variables and provides utility functions
 * for the centralized global management system.
 ****************************************************************************/

#include "vcu_globals.h"
#include "IO_RTC.h"
#include "sensors.h"

//=============================================================================
// GLOBAL VARIABLE DEFINITIONS
//=============================================================================

// System timing variables
ubyte4 timestamp_Precharge = 0;
extern ubyte4 timestamp_SoftBSPD;  // Defined in safety.c
bool prevHVILState = FALSE;

//=============================================================================
// GLOBAL INITIALIZATION
//=============================================================================

void VCU_Globals_Initialize(void)
{
    // Initialize timing variables
    timestamp_Precharge = 0;
    // timestamp_SoftBSPD is initialized in safety.c
    prevHVILState = FALSE;
    
    // Initialize sensor values to safe defaults
    // Note: Actual sensor initialization is handled in sensors.c
}

//=============================================================================
// TIMING UTILITY FUNCTIONS
//=============================================================================

ubyte4 VCU_GetSystemTimeUS(void)
{
    ubyte4 currentTime;
    IO_RTC_StartTime(&currentTime);
    return currentTime;
}

bool VCU_IsTimeElapsed(ubyte4 startTime, ubyte4 duration)
{
    return (IO_RTC_GetTimeUS(startTime) >= duration);
}

//=============================================================================
// SAFETY UTILITY FUNCTIONS
//=============================================================================

bool VCU_IsSystemSafe(void)
{
    // Basic system safety check
    // This can be expanded to include more comprehensive safety checks
    return (Sensor_HVILTerminationSense.sensorValue == TRUE);
}

void VCU_SetSystemFault(bool fault)
{
    // Set system fault state
    // This can be expanded to include fault logging and handling
    if (fault) {
        // Handle fault condition
        // Could set error flags, log fault, etc.
    }
}

//=============================================================================
// CONSTANT VALIDATION FUNCTIONS
//=============================================================================

bool VCU_ValidateConstants(void)
{
    // Validate that all constants are within expected ranges
    bool valid = TRUE;
    
    // Validate timing constants
    if (VCU_MAIN_LOOP_PERIOD_US == 0 || VCU_MAIN_LOOP_PERIOD_US > 50000) {
        valid = FALSE;
    }
    
    // Validate torque constants
    if (VCU_MAX_TORQUE_NM <= VCU_NOMINAL_TORQUE_NM) {
        valid = FALSE;
    }
    
    // Validate scaling factors
    if (VCU_BMS_VOLTAGE_SCALE == 0 || VCU_BMS_CURRENT_SCALE == 0) {
        valid = FALSE;
    }
    
    return valid;
}

//=============================================================================
// DEBUG AND DIAGNOSTIC FUNCTIONS
//=============================================================================

void VCU_PrintSystemInfo(void)
{
    // Print system information for debugging
    // This function can be used during development and testing
    // Implementation depends on available debug output method
}

void VCU_PrintGlobalStatus(void)
{
    // Print status of all global variables
    // Useful for debugging and system monitoring
    // Implementation depends on available debug output method
}
