#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float lookupTable(int velocity, int steering);
float getYaw(int velocity, int steering);

#endif      