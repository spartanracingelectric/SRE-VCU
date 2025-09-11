/*****************************************************************************
 * error_handling.h - Centralized Error Handling System
 ******************************************************************************
 * This file provides a centralized error handling system that replaces
 * the inconsistent error handling patterns found throughout the original codebase.
 ****************************************************************************/

#ifndef _ERROR_HANDLING_H
#define _ERROR_HANDLING_H

#include "IO_Driver.h"
#include "vcuGlobals.h"

//=============================================================================
// ERROR CODE DEFINITIONS
//=============================================================================

// System-level error codes (0x0000 - 0x0FFF)
#define VCU_ERROR_NONE                     0x0000
#define VCU_ERROR_SYSTEM_INIT_FAILED       0x0001
#define VCU_ERROR_MEMORY_ALLOCATION        0x0002
#define VCU_ERROR_HARDWARE_INIT_FAILED     0x0003
#define VCU_ERROR_CRITICAL_TIMING_VIOLATION 0x0004

// Communication error codes (0x1000 - 0x1FFF)
#define VCU_ERROR_CAN_INIT_FAILED          0x1001
#define VCU_ERROR_CAN_COMMUNICATION        0x1002
#define VCU_ERROR_CAN_TIMEOUT              0x1003
#define VCU_ERROR_CAN_FIFO_OVERFLOW        0x1004
#define VCU_ERROR_CAN_INVALID_MESSAGE      0x1005

// Sensor error codes (0x2000 - 0x2FFF)
#define VCU_ERROR_SENSOR_READING           0x2001
#define VCU_ERROR_SENSOR_CALIBRATION       0x2002
#define VCU_ERROR_SENSOR_PLAUSIBILITY      0x2003
#define VCU_ERROR_SENSOR_RANGE_EXCEEDED    0x2004
#define VCU_ERROR_SENSOR_TIMEOUT           0x2005

// Safety error codes (0x3000 - 0x3FFF)
#define VCU_ERROR_SAFETY_VIOLATION         0x3001
#define VCU_ERROR_SAFETY_SHUTDOWN          0x3002
#define VCU_ERROR_SAFETY_TIMEOUT           0x3003
#define VCU_ERROR_SAFETY_SYSTEM_FAULT      0x3004

// Motor control error codes (0x4000 - 0x4FFF)
#define VCU_ERROR_MOTOR_INIT_FAILED        0x4001
#define VCU_ERROR_MOTOR_COMMUNICATION      0x4002
#define VCU_ERROR_MOTOR_FAULT              0x4003
#define VCU_ERROR_MOTOR_TIMEOUT            0x4004
#define VCU_ERROR_MOTOR_OVERCURRENT        0x4005

// Battery management error codes (0x5000 - 0x5FFF)
#define VCU_ERROR_BMS_COMMUNICATION        0x5001
#define VCU_ERROR_BMS_FAULT                0x5002
#define VCU_ERROR_BMS_OVERVOLTAGE          0x5003
#define VCU_ERROR_BMS_UNDERVOLTAGE         0x5004
#define VCU_ERROR_BMS_OVERTEMPERATURE      0x5005

// Calibration error codes (0x6000 - 0x6FFF)
#define VCU_ERROR_CALIBRATION_FAILED       0x6001
#define VCU_ERROR_CALIBRATION_TIMEOUT      0x6002
#define VCU_ERROR_CALIBRATION_INVALID      0x6003
#define VCU_ERROR_CALIBRATION_INTERRUPTED  0x6004

// Utility error codes (0x7000 - 0x7FFF)
#define VCU_ERROR_UTILITY_INVALID_PARAM    0x7001
#define VCU_ERROR_UTILITY_BOUNDS_EXCEEDED  0x7002
#define VCU_ERROR_UTILITY_NULL_POINTER     0x7003
#define VCU_ERROR_UTILITY_DIVISION_BY_ZERO 0x7004

//=============================================================================
// ERROR SEVERITY LEVELS
//=============================================================================

typedef enum {
    VCU_ERROR_SEVERITY_INFO = 0,
    VCU_ERROR_SEVERITY_WARNING = 1,
    VCU_ERROR_SEVERITY_ERROR = 2,
    VCU_ERROR_SEVERITY_CRITICAL = 3,
    VCU_ERROR_SEVERITY_FATAL = 4
} VCU_ErrorSeverity;

//=============================================================================
// ERROR STATE STRUCTURE
//=============================================================================

typedef struct {
    ubyte2 errorCode;
    VCU_ErrorSeverity severity;
    ubyte4 timestamp;
    ubyte1 retryCount;
    bool isActive;
    char description[32];
} VCU_ErrorState;

//=============================================================================
// ERROR HANDLING FUNCTIONS
//=============================================================================

/**
 * Initialize the error handling system
 */
void VCU_Error_Initialize(void);

/**
 * Set an error condition
 * @param errorCode Error code to set
 * @param severity Severity level of the error
 * @param description Optional error description
 */
void VCU_Error_Set(ubyte2 errorCode, VCU_ErrorSeverity severity, const char* description);

/**
 * Clear an error condition
 * @param errorCode Error code to clear
 */
void VCU_Error_Clear(ubyte2 errorCode);

/**
 * Clear all error conditions
 */
void VCU_Error_ClearAll(void);

/**
 * Check if a specific error is active
 * @param errorCode Error code to check
 * @return TRUE if error is active, FALSE otherwise
 */
bool VCU_Error_IsActive(ubyte2 errorCode);

/**
 * Check if any error is active
 * @return TRUE if any error is active, FALSE otherwise
 */
bool VCU_Error_HasAnyError(void);

/**
 * Check if any critical or fatal error is active
 * @return TRUE if critical/fatal error is active, FALSE otherwise
 */
bool VCU_Error_HasCriticalError(void);

/**
 * Get the highest severity active error
 * @return Highest severity level of active errors
 */
VCU_ErrorSeverity VCU_Error_GetHighestSeverity(void);

/**
 * Get the count of active errors
 * @return Number of active errors
 */
ubyte1 VCU_Error_GetActiveCount(void);

/**
 * Get error information
 * @param errorCode Error code to get information for
 * @param errorState Pointer to structure to fill with error information
 * @return TRUE if error found, FALSE otherwise
 */
bool VCU_Error_GetInfo(ubyte2 errorCode, VCU_ErrorState* errorState);

/**
 * Get all active errors
 * @param errorStates Array to fill with active error states
 * @param maxCount Maximum number of errors to return
 * @return Number of active errors returned
 */
ubyte1 VCU_Error_GetAllActive(VCU_ErrorState* errorStates, ubyte1 maxCount);

//=============================================================================
// ERROR HANDLING MACROS
//=============================================================================

// Set error with automatic severity detection
#define VCU_ERROR_SET(errorCode) \
    VCU_Error_Set((errorCode), VCU_ERROR_SEVERITY_ERROR, #errorCode)

// Set error with custom severity
#define VCU_ERROR_SET_SEVERITY(errorCode, severity) \
    VCU_Error_Set((errorCode), (severity), #errorCode)

// Set error with description
#define VCU_ERROR_SET_DESC(errorCode, desc) \
    VCU_Error_Set((errorCode), VCU_ERROR_SEVERITY_ERROR, (desc))

// Check if error is active
#define VCU_ERROR_CHECK(errorCode) \
    VCU_Error_IsActive(errorCode)

// Clear error
#define VCU_ERROR_CLEAR(errorCode) \
    VCU_Error_Clear(errorCode)

// Return if error occurred
#define VCU_ERROR_RETURN_IF(errorCode) \
    do { if (VCU_Error_IsActive(errorCode)) return; } while(0)

// Return value if error occurred
#define VCU_ERROR_RETURN_VAL_IF(errorCode, value) \
    do { if (VCU_Error_IsActive(errorCode)) return (value); } while(0)

//=============================================================================
// ERROR HANDLING UTILITIES
//=============================================================================

/**
 * Get error severity from error code
 * @param errorCode Error code to analyze
 * @return Severity level
 */
VCU_ErrorSeverity VCU_Error_GetSeverityFromCode(ubyte2 errorCode);

/**
 * Get error description from error code
 * @param errorCode Error code to get description for
 * @return Pointer to error description string
 */
const char* VCU_Error_GetDescription(ubyte2 errorCode);

/**
 * Check if error code is critical
 * @param errorCode Error code to check
 * @return TRUE if error is critical, FALSE otherwise
 */
bool VCU_Error_IsCritical(ubyte2 errorCode);

/**
 * Check if error code is fatal
 * @param errorCode Error code to check
 * @return TRUE if error is fatal, FALSE otherwise
 */
bool VCU_Error_IsFatal(ubyte2 errorCode);

/**
 * Get recommended action for error
 * @param errorCode Error code to get action for
 * @return Recommended action string
 */
const char* VCU_Error_GetRecommendedAction(ubyte2 errorCode);

//=============================================================================
// ERROR LOGGING FUNCTIONS
//=============================================================================

/**
 * Log error to system log
 * @param errorCode Error code to log
 * @param additionalInfo Additional information to log
 */
void VCU_Error_Log(ubyte2 errorCode, const char* additionalInfo);

/**
 * Get error log count
 * @return Number of errors logged
 */
ubyte4 VCU_Error_GetLogCount(void);

/**
 * Clear error log
 */
void VCU_Error_ClearLog(void);

#endif // _ERROR_HANDLING_H
