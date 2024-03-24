#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "math.h"

int floorToNearest5(int num)
{
     if (num % 5 != 0) {
        int remainder = num % 5;
        return num - remainder;
    } 
    else 
    {
        return num; 
    }
}
int ceilToNearest5(int num)
{
    if (num % 5 != 0) {
        int remainder = num % 5;
        return num + (5 - remainder);
    } 
    else
    {
        return num;
    }
}