#include <stdio.h>
// The setpoint from the function GOES INTO A PID which should include a saturation check 
// Re-evaluate using slip saturation its not needed we can just add a saturation check at the very end. Still have add PID controller

float sliptarget = 0.2;
float gain = 1/1.2;

//input a slipval for each axel to get the value for each 
float calculateslipadjust(float slipval)
{
    float setpoint=0.0; 
    int enable = 0;
    
    // what is the gain?
    float slip = slipval * gain;
    
    if (slip >= 0) // Positive Slip (Acceleration control)
    {
        if (slip < sliptarget)// I changed the sign here from compared to james algorithm since he flipped the variables for calulating the error
        // i wanted to calculate error using the same order so I changed it since he was not being consistent
        {
            setpoint = sliptarget;
            enable = 1; // Slip Control is active
        }
    }
    else if (slip < 0) // Negative slip (Braking control)
    {
        if (slip > sliptarget * -1)
        {       
            setpoint = sliptarget * -1;
            enable = 1; 
        }
    }
    else
    {
        setpoint = 0.0;
        enable = 0; // Slip Control is inactive
    }

    return setpoint; 

}

