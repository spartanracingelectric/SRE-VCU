/*****************************************************************************
 * common_utils.h - Common Utility Functions
 ******************************************************************************
 * This file provides common utility functions that were duplicated throughout
 * the original codebase. 
 ****************************************************************************/

#ifndef _COMMON_UTILS_H
#define _COMMON_UTILS_H

#include "IO_Driver.h"
#include "vcuGlobals.h"

//=============================================================================
// DATA EXTRACTION UTILITIES
//=============================================================================

/**
 * Extract 16-bit unsigned integer from little-endian byte array
 * @param data Pointer to byte array
 * @param offset Starting offset in array
 * @return 16-bit unsigned integer value
 */
ubyte2 CommonUtils_extractUint16LE(ubyte1* data, ubyte1 offset);

/**
 * Extract 32-bit unsigned integer from little-endian byte array
 * @param data Pointer to byte array
 * @param offset Starting offset in array
 * @return 32-bit unsigned integer value
 */
ubyte4 CommonUtils_extractUint32LE(ubyte1* data, ubyte1 offset);

/**
 * Extract 16-bit signed integer from little-endian byte array
 * @param data Pointer to byte array
 * @param offset Starting offset in array
 * @return 16-bit signed integer value
 */
sbyte2 CommonUtils_extractInt16LE(ubyte1* data, ubyte1 offset);

/**
 * Extract 32-bit signed integer from little-endian byte array
 * @param data Pointer to byte array
 * @param offset Starting offset in array
 * @return 32-bit signed integer value
 */
sbyte4 CommonUtils_extractInt32LE(ubyte1* data, ubyte1 offset);

//=============================================================================
// BIT MANIPULATION UTILITIES
//=============================================================================

/**
 * Extract bit field from byte
 * @param data Source byte
 * @param mask Bit mask for field
 * @param shift Number of bits to shift right
 * @return Extracted bit field value
 */
ubyte1 CommonUtils_extractBitField(ubyte1 data, ubyte1 mask, ubyte1 shift);

/**
 * Set bit field in byte
 * @param data Pointer to destination byte
 * @param mask Bit mask for field
 * @param shift Number of bits to shift left
 * @param value Value to set
 */
void CommonUtils_setBitField(ubyte1* data, ubyte1 mask, ubyte1 shift, ubyte1 value);

/**
 * Check if specific bit is set
 * @param data Source byte
 * @param bit Bit position to check
 * @return TRUE if bit is set, FALSE otherwise
 */
bool CommonUtils_isBitSet(ubyte1 data, ubyte1 bit);

/**
 * Set specific bit
 * @param data Pointer to destination byte
 * @param bit Bit position to set
 */
void CommonUtils_setBit(ubyte1* data, ubyte1 bit);

/**
 * Clear specific bit
 * @param data Pointer to destination byte
 * @param bit Bit position to clear
 */
void CommonUtils_clearBit(ubyte1* data, ubyte1 bit);

/**
 * Toggle specific bit
 * @param data Pointer to destination byte
 * @param bit Bit position to toggle
 */
void CommonUtils_toggleBit(ubyte1* data, ubyte1 bit);

//=============================================================================
// DATA SCALING UTILITIES
//=============================================================================

/**
 * Apply scaling and offset to raw value
 * @param rawValue Raw input value
 * @param scale Scaling factor
 * @param offset Offset value
 * @return Scaled float value
 */
float4 CommonUtils_applyScaling(ubyte4 rawValue, float4 scale, float4 offset);

/**
 * Apply inverse scaling to get raw value from scaled value
 * @param scaledValue Scaled input value
 * @param scale Scaling factor
 * @param offset Offset value
 * @return Raw integer value
 */
ubyte4 CommonUtils_applyInverseScaling(float4 scaledValue, float4 scale, float4 offset);

/**
 * Convert raw value to percentage (0.0 to 1.0)
 * @param value Input value
 * @param minValue Minimum value for range
 * @param maxValue Maximum value for range
 * @param clampToRange TRUE to clamp result to 0.0-1.0 range
 * @return Percentage value (0.0 to 1.0)
 */
float4 CommonUtils_valueToPercent(ubyte4 value, ubyte4 minValue, ubyte4 maxValue, bool clampToRange);

/**
 * Convert percentage to raw value
 * @param percent Percentage value (0.0 to 1.0)
 * @param minValue Minimum value for range
 * @param maxValue Maximum value for range
 * @return Raw integer value
 */
ubyte4 CommonUtils_percentToValue(float4 percent, ubyte4 minValue, ubyte4 maxValue);

//=============================================================================
// ARRAY UTILITIES
//=============================================================================

/**
 * Copy array with bounds checking
 * @param dest Destination array
 * @param src Source array
 * @param count Number of elements to copy
 * @param maxDestSize Maximum size of destination array
 * @return TRUE if successful, FALSE if bounds exceeded
 */
bool CommonUtils_safeArrayCopy(ubyte1* dest, const ubyte1* src, ubyte2 count, ubyte2 maxDestSize);

/**
 * Clear array to zero
 * @param array Pointer to array
 * @param count Number of elements to clear
 */
void CommonUtils_clearArray(ubyte1* array, ubyte2 count);

/**
 * Compare two arrays
 * @param array1 First array
 * @param array2 Second array
 * @param count Number of elements to compare
 * @return TRUE if arrays are equal, FALSE otherwise
 */
bool CommonUtils_compareArrays(const ubyte1* array1, const ubyte1* array2, ubyte2 count);

//=============================================================================
// VALIDATION UTILITIES
//=============================================================================

/**
 * Check if value is within range
 * @param value Value to check
 * @param minValue Minimum allowed value
 * @param maxValue Maximum allowed value
 * @return TRUE if value is in range, FALSE otherwise
 */
bool CommonUtils_isInRange(ubyte4 value, ubyte4 minValue, ubyte4 maxValue);

/**
 * Clamp value to range
 * @param value Value to clamp
 * @param minValue Minimum allowed value
 * @param maxValue Maximum allowed value
 * @return Clamped value
 */
ubyte4 CommonUtils_clampToRange(ubyte4 value, ubyte4 minValue, ubyte4 maxValue);

/**
 * Check if two values are within tolerance
 * @param value1 First value
 * @param value2 Second value
 * @param tolerance Maximum allowed difference
 * @return TRUE if values are within tolerance, FALSE otherwise
 */
bool CommonUtils_isWithinTolerance(ubyte4 value1, ubyte4 value2, ubyte4 tolerance);

//=============================================================================
// CONVERSION UTILITIES
//=============================================================================

/**
 * Convert RPM to MPH
 * @param rpm Revolutions per minute
 * @return Speed in miles per hour
 */
float4 CommonUtils_rpmToMph(float4 rpm);

/**
 * Convert frequency to RPM
 * @param freq Frequency in Hz
 * @return Revolutions per minute
 */
float4 CommonUtils_freqToRpm(float4 freq);

/**
 * Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
float4 CommonUtils_degreesToRadians(float4 degrees);

/**
 * Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
float4 CommonUtils_radiansToDegrees(float4 radians);

//=============================================================================
// ERROR HANDLING UTILITIES
//=============================================================================

/**
 * Set error code with optional message
 * @param errorCode Error code to set
 * @param message Optional error message (can be NULL)
 */
void CommonUtils_setError(ubyte2 errorCode, const char* message);

/**
 * Get last error code
 * @return Last error code
 */
ubyte2 CommonUtils_getLastError(void);

/**
 * Clear error state
 */
void CommonUtils_clearError(void);

/**
 * Check if error occurred
 * @return TRUE if error occurred, FALSE otherwise
 */
bool CommonUtils_hasError(void);

//=============================================================================
// MACROS FOR COMMON OPERATIONS
//=============================================================================

// Safe array access with bounds checking
#define SAFE_ARRAY_ACCESS(array, index, maxSize) \
    (((index) < (maxSize)) ? (array)[(index)] : 0)

// Safe pointer dereference
#define SAFE_PTR_DEREF(ptr, defaultVal) \
    ((ptr) != NULL ? *(ptr) : (defaultVal))

// Minimum of two values
#define MIN(a, b) \
    (((a) < (b)) ? (a) : (b))

// Maximum of two values
#define MAX(a, b) \
    (((a) > (b)) ? (a) : (b))

// Absolute value
#define ABS(x) \
    (((x) < 0) ? -(x) : (x))

// Round to nearest integer
#define ROUND(x) \
    ((ubyte4)((x) + 0.5f))

// Clamp value to range
#define CLAMP(value, min, max) \
    (MIN(MAX((value), (min)), (max)))

#endif // _COMMON_UTILS_H
