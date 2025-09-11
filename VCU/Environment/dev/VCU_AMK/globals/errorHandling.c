/*****************************************************************************
 * error_handling.c - Centralized Error Handling System Implementation
 * Optimized Version
 ******************************************************************************
 * This file implements the centralized error handling system that provides
 * consistent error management throughout the VCU system.
 ****************************************************************************/

#include "errorHandling.h"
#include "IO_RTC.h"
#include <string.h>

//=============================================================================
// PRIVATE CONSTANTS
//=============================================================================

#define VCU_MAX_ERRORS 32
#define VCU_ERROR_LOG_SIZE 128

//=============================================================================
// PRIVATE VARIABLES
//=============================================================================

static VCU_ErrorState errorStates[VCU_MAX_ERRORS];
static ubyte1 errorCount = 0;
static ubyte4 errorLogCount = 0;
static bool systemInitialized = FALSE;

//=============================================================================
// PRIVATE FUNCTION PROTOTYPES
//=============================================================================

static void VCU_Error_AddToLog(ubyte2 errorCode, const char* description);
static VCU_ErrorSeverity VCU_Error_DetermineSeverity(ubyte2 errorCode);
static const char* VCU_Error_GetCodeDescription(ubyte2 errorCode);

//=============================================================================
// PUBLIC FUNCTIONS
//=============================================================================

void VCU_Error_Initialize(void)
{
    // Clear all error states
    memset(errorStates, 0, sizeof(errorStates));
    errorCount = 0;
    errorLogCount = 0;
    systemInitialized = TRUE;
}

void VCU_Error_Set(ubyte2 errorCode, VCU_ErrorSeverity severity, const char* description)
{
    if (!systemInitialized) {
        return;
    }
    
    // Find existing error or empty slot
    ubyte1 index = 0;
    bool found = FALSE;
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        if (errorStates[i].errorCode == errorCode) {
            index = i;
            found = TRUE;
            break;
        }
        if (errorStates[i].errorCode == 0 && !found) {
            index = i;
        }
    }
    
    // Set error state
    errorStates[index].errorCode = errorCode;
    errorStates[index].severity = severity;
    errorStates[index].timestamp = VCU_GetSystemTimeUS();
    errorStates[index].isActive = TRUE;
    errorStates[index].retryCount++;
    
    // Copy description
    if (description != NULL) {
        strncpy(errorStates[index].description, description, sizeof(errorStates[index].description) - 1);
        errorStates[index].description[sizeof(errorStates[index].description) - 1] = '\0';
    } else {
        strncpy(errorStates[index].description, VCU_Error_GetCodeDescription(errorCode), sizeof(errorStates[index].description) - 1);
        errorStates[index].description[sizeof(errorStates[index].description) - 1] = '\0';
    }
    
    // Update count if new error
    if (!found) {
        errorCount++;
    }
    
    // Add to log
    VCU_Error_AddToLog(errorCode, errorStates[index].description);
}

void VCU_Error_Clear(ubyte2 errorCode)
{
    if (!systemInitialized) {
        return;
    }
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        if (errorStates[i].errorCode == errorCode) {
            errorStates[i].isActive = FALSE;
            errorStates[i].retryCount = 0;
            errorCount--;
            break;
        }
    }
}

void VCU_Error_ClearAll(void)
{
    if (!systemInitialized) {
        return;
    }
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        errorStates[i].isActive = FALSE;
        errorStates[i].retryCount = 0;
    }
    errorCount = 0;
}

bool VCU_Error_IsActive(ubyte2 errorCode)
{
    if (!systemInitialized) {
        return FALSE;
    }
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        if (errorStates[i].errorCode == errorCode && errorStates[i].isActive) {
            return TRUE;
        }
    }
    return FALSE;
}

bool VCU_Error_HasAnyError(void)
{
    return (errorCount > 0);
}

bool VCU_Error_HasCriticalError(void)
{
    if (!systemInitialized) {
        return FALSE;
    }
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        if (errorStates[i].isActive && 
            (errorStates[i].severity == VCU_ERROR_SEVERITY_CRITICAL || 
             errorStates[i].severity == VCU_ERROR_SEVERITY_FATAL)) {
            return TRUE;
        }
    }
    return FALSE;
}

VCU_ErrorSeverity VCU_Error_GetHighestSeverity(void)
{
    if (!systemInitialized) {
        return VCU_ERROR_SEVERITY_INFO;
    }
    
    VCU_ErrorSeverity highest = VCU_ERROR_SEVERITY_INFO;
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        if (errorStates[i].isActive && errorStates[i].severity > highest) {
            highest = errorStates[i].severity;
        }
    }
    
    return highest;
}

ubyte1 VCU_Error_GetActiveCount(void)
{
    return errorCount;
}

bool VCU_Error_GetInfo(ubyte2 errorCode, VCU_ErrorState* errorState)
{
    if (!systemInitialized || errorState == NULL) {
        return FALSE;
    }
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS; i++) {
        if (errorStates[i].errorCode == errorCode) {
            *errorState = errorStates[i];
            return TRUE;
        }
    }
    
    return FALSE;
}

ubyte1 VCU_Error_GetAllActive(VCU_ErrorState* errorStates_out, ubyte1 maxCount)
{
    if (!systemInitialized || errorStates_out == NULL) {
        return 0;
    }
    
    ubyte1 count = 0;
    
    for (ubyte1 i = 0; i < VCU_MAX_ERRORS && count < maxCount; i++) {
        if (errorStates[i].isActive) {
            errorStates_out[count] = errorStates[i];
            count++;
        }
    }
    
    return count;
}

//=============================================================================
// ERROR HANDLING UTILITIES
//=============================================================================

VCU_ErrorSeverity VCU_Error_GetSeverityFromCode(ubyte2 errorCode)
{
    return VCU_Error_DetermineSeverity(errorCode);
}

const char* VCU_Error_GetDescription(ubyte2 errorCode)
{
    return VCU_Error_GetCodeDescription(errorCode);
}

bool VCU_Error_IsCritical(ubyte2 errorCode)
{
    return (VCU_Error_DetermineSeverity(errorCode) >= VCU_ERROR_SEVERITY_CRITICAL);
}

bool VCU_Error_IsFatal(ubyte2 errorCode)
{
    return (VCU_Error_DetermineSeverity(errorCode) == VCU_ERROR_SEVERITY_FATAL);
}

const char* VCU_Error_GetRecommendedAction(ubyte2 errorCode)
{
    // Return recommended action based on error code
    switch (errorCode) {
        case VCU_ERROR_MEMORY_ALLOCATION:
            return "Check available memory and restart system";
        case VCU_ERROR_CAN_COMMUNICATION:
            return "Check CAN bus connections and restart communication";
        case VCU_ERROR_SENSOR_CALIBRATION:
            return "Re-run sensor calibration procedure";
        case VCU_ERROR_SAFETY_VIOLATION:
            return "Check safety system and clear faults";
        case VCU_ERROR_MOTOR_FAULT:
            return "Check motor connections and clear motor faults";
        case VCU_ERROR_BMS_FAULT:
            return "Check battery management system and clear BMS faults";
        default:
            return "Check system logs and contact support";
    }
}

//=============================================================================
// ERROR LOGGING FUNCTIONS
//=============================================================================

void VCU_Error_Log(ubyte2 errorCode, const char* additionalInfo)
{
    if (!systemInitialized) {
        return;
    }
    
    // In a real implementation, this would write to a log file or send to debug output
    // For now, we just increment the log count
    errorLogCount++;
    
    // Could also store log entries in a circular buffer for later retrieval
}

ubyte4 VCU_Error_GetLogCount(void)
{
    return errorLogCount;
}

void VCU_Error_ClearLog(void)
{
    errorLogCount = 0;
}

//=============================================================================
// PRIVATE FUNCTIONS
//=============================================================================

static void VCU_Error_AddToLog(ubyte2 errorCode, const char* description)
{
    // Add error to log (implementation depends on available logging mechanism)
    VCU_Error_Log(errorCode, description);
}

static VCU_ErrorSeverity VCU_Error_DetermineSeverity(ubyte2 errorCode)
{
    // Determine severity based on error code ranges
    if (errorCode >= 0x0000 && errorCode <= 0x0FFF) {
        // System errors
        if (errorCode == VCU_ERROR_CRITICAL_TIMING_VIOLATION) {
            return VCU_ERROR_SEVERITY_FATAL;
        }
        return VCU_ERROR_SEVERITY_ERROR;
    }
    else if (errorCode >= 0x1000 && errorCode <= 0x1FFF) {
        // Communication errors
        return VCU_ERROR_SEVERITY_ERROR;
    }
    else if (errorCode >= 0x2000 && errorCode <= 0x2FFF) {
        // Sensor errors
        if (errorCode == VCU_ERROR_SENSOR_PLAUSIBILITY) {
            return VCU_ERROR_SEVERITY_CRITICAL;
        }
        return VCU_ERROR_SEVERITY_WARNING;
    }
    else if (errorCode >= 0x3000 && errorCode <= 0x3FFF) {
        // Safety errors
        return VCU_ERROR_SEVERITY_CRITICAL;
    }
    else if (errorCode >= 0x4000 && errorCode <= 0x4FFF) {
        // Motor control errors
        if (errorCode == VCU_ERROR_MOTOR_OVERCURRENT) {
            return VCU_ERROR_SEVERITY_CRITICAL;
        }
        return VCU_ERROR_SEVERITY_ERROR;
    }
    else if (errorCode >= 0x5000 && errorCode <= 0x5FFF) {
        // Battery management errors
        if (errorCode == VCU_ERROR_BMS_OVERVOLTAGE || errorCode == VCU_ERROR_BMS_OVERTEMPERATURE) {
            return VCU_ERROR_SEVERITY_CRITICAL;
        }
        return VCU_ERROR_SEVERITY_ERROR;
    }
    else if (errorCode >= 0x6000 && errorCode <= 0x6FFF) {
        // Calibration errors
        return VCU_ERROR_SEVERITY_WARNING;
    }
    else if (errorCode >= 0x7000 && errorCode <= 0x7FFF) {
        // Utility errors
        return VCU_ERROR_SEVERITY_WARNING;
    }
    
    return VCU_ERROR_SEVERITY_ERROR;
}

static const char* VCU_Error_GetCodeDescription(ubyte2 errorCode)
{
    // Return description based on error code
    switch (errorCode) {
        case VCU_ERROR_NONE:
            return "No error";
        case VCU_ERROR_SYSTEM_INIT_FAILED:
            return "System initialization failed";
        case VCU_ERROR_MEMORY_ALLOCATION:
            return "Memory allocation failed";
        case VCU_ERROR_HARDWARE_INIT_FAILED:
            return "Hardware initialization failed";
        case VCU_ERROR_CRITICAL_TIMING_VIOLATION:
            return "Critical timing violation";
        case VCU_ERROR_CAN_INIT_FAILED:
            return "CAN initialization failed";
        case VCU_ERROR_CAN_COMMUNICATION:
            return "CAN communication error";
        case VCU_ERROR_CAN_TIMEOUT:
            return "CAN timeout";
        case VCU_ERROR_CAN_FIFO_OVERFLOW:
            return "CAN FIFO overflow";
        case VCU_ERROR_CAN_INVALID_MESSAGE:
            return "Invalid CAN message";
        case VCU_ERROR_SENSOR_READING:
            return "Sensor reading error";
        case VCU_ERROR_SENSOR_CALIBRATION:
            return "Sensor calibration error";
        case VCU_ERROR_SENSOR_PLAUSIBILITY:
            return "Sensor plausibility check failed";
        case VCU_ERROR_SENSOR_RANGE_EXCEEDED:
            return "Sensor range exceeded";
        case VCU_ERROR_SENSOR_TIMEOUT:
            return "Sensor timeout";
        case VCU_ERROR_SAFETY_VIOLATION:
            return "Safety violation";
        case VCU_ERROR_SAFETY_SHUTDOWN:
            return "Safety shutdown";
        case VCU_ERROR_SAFETY_TIMEOUT:
            return "Safety timeout";
        case VCU_ERROR_SAFETY_SYSTEM_FAULT:
            return "Safety system fault";
        case VCU_ERROR_MOTOR_INIT_FAILED:
            return "Motor initialization failed";
        case VCU_ERROR_MOTOR_COMMUNICATION:
            return "Motor communication error";
        case VCU_ERROR_MOTOR_FAULT:
            return "Motor fault";
        case VCU_ERROR_MOTOR_TIMEOUT:
            return "Motor timeout";
        case VCU_ERROR_MOTOR_OVERCURRENT:
            return "Motor overcurrent";
        case VCU_ERROR_BMS_COMMUNICATION:
            return "BMS communication error";
        case VCU_ERROR_BMS_FAULT:
            return "BMS fault";
        case VCU_ERROR_BMS_OVERVOLTAGE:
            return "BMS overvoltage";
        case VCU_ERROR_BMS_UNDERVOLTAGE:
            return "BMS undervoltage";
        case VCU_ERROR_BMS_OVERTEMPERATURE:
            return "BMS overtemperature";
        case VCU_ERROR_CALIBRATION_FAILED:
            return "Calibration failed";
        case VCU_ERROR_CALIBRATION_TIMEOUT:
            return "Calibration timeout";
        case VCU_ERROR_CALIBRATION_INVALID:
            return "Invalid calibration";
        case VCU_ERROR_CALIBRATION_INTERRUPTED:
            return "Calibration interrupted";
        case VCU_ERROR_UTILITY_INVALID_PARAM:
            return "Invalid parameter";
        case VCU_ERROR_UTILITY_BOUNDS_EXCEEDED:
            return "Bounds exceeded";
        case VCU_ERROR_UTILITY_NULL_POINTER:
            return "Null pointer";
        case VCU_ERROR_UTILITY_DIVISION_BY_ZERO:
            return "Division by zero";
        default:
            return "Unknown error";
    }
}
