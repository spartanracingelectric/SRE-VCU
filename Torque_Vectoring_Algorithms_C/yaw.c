#include <stdio.h>
#include "yawhashtable.h"


float calibratedYaw=0.0;
float yawerror=0.0;
float gainval= 1.618;
float glob_steering =0.0;

int floorToNearest5(int num) {
    if (num % 5 != 0) {
        int remainder = num % 5;
        return num - remainder;
    } 
    else 
    {
        return num; 
    }
}

int ceilToNearest5(int num) {
    if (num % 5 != 0) {
        int remainder = num % 5;
        return num + (5 - remainder);
    } 
    else
    {
        return num;
    }
}

// Put in velocity and steering values and get value from the lookupTable 
float lookupTable(int velocity, int steering) {
    // x-axis: velocity
    // y-axis: steering 
    int abssteer = steering; 

    if(abssteer<0)// accounts for right hand turns which give negative values 
    {
        abssteer = -1*steering; 
    }
     
    glob_steering = abssteer;

    HashTable* table = createHashTable();
    initializeHashTable(table);
    float yaw = get(table,velocity,abssteer);
    destroyHashTable(table);
    return yaw;
}

// Uses double linear interpolation for x and y axis
float getYaw(int velocity, int steering) {
    int velocityFloor = floorToNearest5(velocity);
    int velocityCeiling = ceilToNearest5(velocity);

    int steeringFloor = floorToNearest5(steering);
    int steeringCeiling = ceilToNearest5(steering);

    float floorFloor = lookupTable(velocityFloor, steeringFloor);
    float ceilingFloor = lookupTable(velocityCeiling, steeringFloor);

    float floorCeiling = lookupTable(velocityFloor, steeringCeiling);
    float ceilingCeiling = lookupTable(velocityCeiling, steeringCeiling);

    float horizontal_Interp = (((ceilingFloor - floorFloor) / 5) + ((ceilingCeiling - floorCeiling) / 5)) / 2;
    float vertical_Interp = (((floorCeiling - floorFloor) / 5) + ((ceilingCeiling - ceilingFloor) / 5)) / 2;

    int gainValueHoriz = velocity % 5;
    int gainValueVertical = steering % 5;

     calibratedYaw = (gainValueHoriz * horizontal_Interp) + (gainValueVertical * vertical_Interp) + floorFloor;
     

    return calibratedYaw;
}

//the pid method goes in here to get the final yawrate value 
//pid outputs yawmoment(yaw torque val )
float getYawRateError(int velocity, int steering, int targetYaw)
{
    float yaw = getYaw(velocity, steering) * gainval;//what is the gain value?
    yawerror = yaw - targetYaw;
    
    return yawerror;
}

int main() {
   HashTable* table = createHashTable();
    initializeHashTable(table);
    
    // Retrieve values
    printf("Value for (velocity = %d, steering = %d): %.2f\n", 5, 5, lookupTable(5,5));
    
    // Destroy the hash table

    return 0;
}
