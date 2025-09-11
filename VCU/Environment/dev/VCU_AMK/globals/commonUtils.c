/*****************************************************************************
 * common_utils.c - Common Utility Functions Implementation
 ******************************************************************************
 * This file implements the common utility functions that eliminate code
 * duplication throughout the system.
 ****************************************************************************/

#include "commonUtils.h"
#include "IO_RTC.h"
#include <string.h>

//=============================================================================
// PRIVATE VARIABLES
//=============================================================================

static ubyte2 lastErrorCode = 0;
static bool errorOccurred = FALSE;

//=============================================================================
// DATA EXTRACTION UTILITIES
//=============================================================================

ubyte2 CommonUtils_extractUint16LE(ubyte1* data, ubyte1 offset)
{
    if (data == NULL) {
        CommonUtils_setError(0x1001, "NULL data pointer in extractUint16LE");
        return 0;
    }
    
    return (ubyte2)((data[offset + 1] << 8) | data[offset]);
}

ubyte4 CommonUtils_extractUint32LE(ubyte1* data, ubyte1 offset)
{
    if (data == NULL) {
        CommonUtils_setError(0x1002, "NULL data pointer in extractUint32LE");
        return 0;
    }
    
    return (ubyte4)((data[offset + 3] << 24) |
                   (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) |
                   data[offset]);
}

sbyte2 CommonUtils_extractInt16LE(ubyte1* data, ubyte1 offset)
{
    if (data == NULL) {
        CommonUtils_setError(0x1003, "NULL data pointer in extractInt16LE");
        return 0;
    }
    
    return (sbyte2)((data[offset + 1] << 8) | data[offset]);
}

sbyte4 CommonUtils_extractInt32LE(ubyte1* data, ubyte1 offset)
{
    if (data == NULL) {
        CommonUtils_setError(0x1004, "NULL data pointer in extractInt32LE");
        return 0;
    }
    
    return (sbyte4)((data[offset + 3] << 24) |
                   (data[offset + 2] << 16) |
                   (data[offset + 1] << 8) |
                   data[offset]);
}

//=============================================================================
// BIT MANIPULATION UTILITIES
//=============================================================================

ubyte1 CommonUtils_extractBitField(ubyte1 data, ubyte1 mask, ubyte1 shift)
{
    return (data & mask) >> shift;
}

void CommonUtils_setBitField(ubyte1* data, ubyte1 mask, ubyte1 shift, ubyte1 value)
{
    if (data == NULL) {
        CommonUtils_setError(0x1005, "NULL data pointer in setBitField");
        return;
    }
    
    *data = (*data & ~mask) | ((value << shift) & mask);
}

bool CommonUtils_isBitSet(ubyte1 data, ubyte1 bit)
{
    return (data & (1 << bit)) != 0;
}

void CommonUtils_setBit(ubyte1* data, ubyte1 bit)
{
    if (data == NULL) {
        CommonUtils_setError(0x1006, "NULL data pointer in setBit");
        return;
    }
    
    *data |= (1 << bit);
}

void CommonUtils_clearBit(ubyte1* data, ubyte1 bit)
{
    if (data == NULL) {
        CommonUtils_setError(0x1007, "NULL data pointer in clearBit");
        return;
    }
    
    *data &= ~(1 << bit);
}

void CommonUtils_toggleBit(ubyte1* data, ubyte1 bit)
{
    if (data == NULL) {
        CommonUtils_setError(0x1008, "NULL data pointer in toggleBit");
        return;
    }
    
    *data ^= (1 << bit);
}

//=============================================================================
// DATA SCALING UTILITIES
//=============================================================================

float4 CommonUtils_applyScaling(ubyte4 rawValue, float4 scale, float4 offset)
{
    return ((float4)rawValue * scale) + offset;
}

ubyte4 CommonUtils_applyInverseScaling(float4 scaledValue, float4 scale, float4 offset)
{
    if (scale == 0.0f) {
        CommonUtils_setError(0x1009, "Zero scale factor in applyInverseScaling");
        return 0;
    }
    
    return (ubyte4)((scaledValue - offset) / scale);
}

float4 CommonUtils_valueToPercent(ubyte4 value, ubyte4 minValue, ubyte4 maxValue, bool clampToRange)
{
    if (maxValue <= minValue) {
        CommonUtils_setError(0x100A, "Invalid range in valueToPercent");
        return 0.0f;
    }
    
    float4 percent = ((float4)(value - minValue)) / ((float4)(maxValue - minValue));
    
    if (clampToRange) {
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 1.0f) percent = 1.0f;
    }
    
    return percent;
}

ubyte4 CommonUtils_percentToValue(float4 percent, ubyte4 minValue, ubyte4 maxValue)
{
    if (maxValue <= minValue) {
        CommonUtils_setError(0x100B, "Invalid range in percentToValue");
        return minValue;
    }
    
    // Clamp percent to valid range
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 1.0f) percent = 1.0f;
    
    return minValue + (ubyte4)(percent * (maxValue - minValue));
}

//=============================================================================
// ARRAY UTILITIES
//=============================================================================

bool CommonUtils_safeArrayCopy(ubyte1* dest, const ubyte1* src, ubyte2 count, ubyte2 maxDestSize)
{
    if (dest == NULL || src == NULL) {
        CommonUtils_setError(0x100C, "NULL pointer in safeArrayCopy");
        return FALSE;
    }
    
    if (count > maxDestSize) {
        CommonUtils_setError(0x100D, "Count exceeds destination size in safeArrayCopy");
        return FALSE;
    }
    
    memcpy(dest, src, count);
    return TRUE;
}

void CommonUtils_clearArray(ubyte1* array, ubyte2 count)
{
    if (array == NULL) {
        CommonUtils_setError(0x100E, "NULL array pointer in clearArray");
        return;
    }
    
    memset(array, 0, count);
}

bool CommonUtils_compareArrays(const ubyte1* array1, const ubyte1* array2, ubyte2 count)
{
    if (array1 == NULL || array2 == NULL) {
        CommonUtils_setError(0x100F, "NULL pointer in compareArrays");
        return FALSE;
    }
    
    return (memcmp(array1, array2, count) == 0);
}

//=============================================================================
// VALIDATION UTILITIES
//=============================================================================

bool CommonUtils_isInRange(ubyte4 value, ubyte4 minValue, ubyte4 maxValue)
{
    return (value >= minValue && value <= maxValue);
}

ubyte4 CommonUtils_clampToRange(ubyte4 value, ubyte4 minValue, ubyte4 maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

bool CommonUtils_isWithinTolerance(ubyte4 value1, ubyte4 value2, ubyte4 tolerance)
{
    ubyte4 diff = (value1 > value2) ? (value1 - value2) : (value2 - value1);
    return (diff <= tolerance);
}

//=============================================================================
// CONVERSION UTILITIES
//=============================================================================

float4 CommonUtils_rpmToMph(float4 rpm)
{
    // Assuming 16 bumps per revolution and 16 inch wheel diameter
    // 63360 inches per mile
    const float4 inchesPerMile = 63360.0f;
    const float4 wheelCircumference = (float4)VCU_WHEEL_DIAMETER_D * 3.14159f;
    const float4 bumpsPerMile = inchesPerMile / wheelCircumference;
    
    return (rpm * 60.0f) / bumpsPerMile; // Convert RPM to MPH
}

float4 CommonUtils_freqToRpm(float4 freq)
{
    return freq * 60.0f; // Convert Hz to RPM
}

float4 CommonUtils_degreesToRadians(float4 degrees)
{
    return degrees * (3.14159f / 180.0f);
}

float4 CommonUtils_radiansToDegrees(float4 radians)
{
    return radians * (180.0f / 3.14159f);
}

//=============================================================================
// ERROR HANDLING UTILITIES
//=============================================================================

void CommonUtils_setError(ubyte2 errorCode, const char* message)
{
    lastErrorCode = errorCode;
    errorOccurred = TRUE;
    
    // In a real implementation, you might want to log the error message
    // For now, we just set the error code
}

ubyte2 CommonUtils_getLastError(void)
{
    return lastErrorCode;
}

void CommonUtils_clearError(void)
{
    lastErrorCode = 0;
    errorOccurred = FALSE;
}

bool CommonUtils_hasError(void)
{
    return errorOccurred;
}
