/*****************************************************************************
 * mathFunctions.c - Math functionality
 * Initial Author: Rusty P
 ******************************************************************************
 * Specialized math control functions
 ****************************************************************************/

#include <stdarg.h>
#include <math.h>
#include "mathFunctions.h"
#include "IO_Driver.h"
#include "IO_RTC.h"

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* Special features:
*   - Handles cases where "start" is less than "end" (value goes backwards)
* Automatically compensates for reverse-direction values: When min > max, value is assumed to travel in reverse direction.
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
* If range == 0, then 0 will be returned.  (Safety: in case of problem during regen torque calculation)
-------------------------------------------------------------------*/
float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly)
{
    float4 retVal = (value - start) / (end - start);

    if (zeroToOneOnly == TRUE)
    {
        if (retVal < 0)
        {
            retVal = 0;
        }
        if (retVal > 1)
        {
            retVal = 1;
        }
    }

    return retVal;
}

/**********************************************************************/ /**
 *
 * \brief Decides whether a blinking light should be on or off based on
 *        the system clock and period given
 *
 * \param timestamp     A timestamp from IO_RTC for when the blink was started
 * \param highPeriod    How long the blink should stay high, in ms (NOT ns)
 * \param lowPeriod     How long the blink should stay low -- NOT IMPLEMENTED;
 *                      Always the same as highPeriod
 *
 * \return boolean:
 * \retval TRUE      blink should be high
 * \retval FALSE     blink should be low
 *
 ***************************************************************************
 *
 * \remarks
 *   Blablabla
 *
 ***************************************************************************/
bool blink(ubyte4 *clock, ubyte2 highPeriod)
{
    //time passed since the start of the blinks divided by the period to get
    //count.
    //count % 2 gets the current state the blink should be in 0 or <0
    ubyte4 count = IO_RTC_GetTimeUS(*clock) / highPeriod;
    //removes decimal places. there may be a better way to do this but I got lazy
    count = count - count % 1;

    return !(count / highPeriod) % 2;
}
