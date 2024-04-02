#include <stdio.h>
#include "yawhashtable.h"
#include "yaw.h"


float calibratedYaw= 0.0;
float gainval= 1.618;

// Put in velocity and steering values and get value from the lookupTable 
float lookupTable(int velocity, int steering, HashTable* yaw_hashtable) {
    // x-axis: velocity
    // y-axis: steering 
    int abssteer = steering; 

    if(abssteer<0)// accounts for right hand turns which give negative values 
    {
        abssteer = -1*steering; 
    }
     
// Instead of Initalizing the hashtable here we do it ouside the while main loop
// and then free the memory with the destroyHashTable method after 

   // HashTable* table = createHashTable();
   // initializeHashTable(table);
    float yaw = get(yaw_hashtable,velocity,abssteer);
    //destroyHashTable(table);
    return yaw;
}

// Uses double linear interpolation for x and y axis
float getYaw(int velocity, int steering, HashTable* yaw_hashtable) {
    int velocityFloor = floorToNearest5(velocity);
    int velocityCeiling = ceilToNearest5(velocity);

    int steeringFloor = floorToNearest5(steering);
    int steeringCeiling = ceilToNearest5(steering);

    float floorFloor = lookupTable(velocityFloor, steeringFloor, yaw_hashtable);
    float ceilingFloor = lookupTable(velocityCeiling, steeringFloor, yaw_hashtable);

    float floorCeiling = lookupTable(velocityFloor, steeringCeiling, yaw_hashtable);
    float ceilingCeiling = lookupTable(velocityCeiling, steeringCeiling, yaw_hashtable);

    float horizontal_Interp = (((ceilingFloor - floorFloor) / 5) + ((ceilingCeiling - floorCeiling) / 5)) / 2;
    float vertical_Interp = (((floorCeiling - floorFloor) / 5) + ((ceilingCeiling - ceilingFloor) / 5)) / 2;

    int gainValueHoriz = velocity % 5;
    int gainValueVertical = steering % 5;

    calibratedYaw = (gainValueHoriz * horizontal_Interp) + (gainValueVertical * vertical_Interp) + floorFloor;

    return calibratedYaw;
}
