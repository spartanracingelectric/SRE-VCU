#include <stdio.h>

float rw = 0.2032;  // wheel radius
float tw = 1.2;     // average track width(meters)
float alpha = (0.2032) / (0.5*1.2);  // Constant from wheel radius and track width; converts Mz into forces (instead of torques)
// 1.34 = trackwidth 
// checking validity of this 
int GR = 13;  // Gear ratio
int motorPk = 21;  // Peak torque output of motor

float Tf_FL = 0.0; // Tire friction Front Left
float Tf_FR = 0.0; // Tire friction Front Right 
float Tf_RL = 0.0; // Tire friction Rear Left
float Tf_RR = 0.0; // Tire friction Rear Right 

float momentBal = 0.0;// moment of balanace 
float front_bias_percent = 0.0;// < 0.5 = rear biased; >=0.5 = front biased
float torque_percent = 0.0;//torque in percentage 
float tractDemand = 0.0;// 

float torqueFL=0.0;
float torqueFR=0.0;
float torqueRL=0.0;
float torqueRR=0.0;

// Declare pointers for tire coefficients
// these pointers are going to be fed into the function as parameters and thier value will be sent 
// to the corresponding tire coefficent float values(Tf_FL)
float *ptr_Tf_FL = &Tf_FL;
float *ptr_Tf_FR = &Tf_FR;
float *ptr_Tf_RL = &Tf_RL;
float *ptr_Tf_RR = &Tf_RR;

void setfrontbiaspercent(float FLFz, float FRFz, float RLFz, float RRFz) 
{
    front_bias_percent = (FLFz + FRFz) / (FLFz + FRFz + RLFz + RRFz);
}

// sets the torque for each axel 
void set_torque_distribution(float *Outer_Rear, float *Outer_Front, float *Inner_Front, float *Inner_Rear, int frontbias_bool, float tractDemand)
{
    *Outer_Rear = tractDemand;
    if (frontbias_bool == 1)//front biased 
    {
        *Outer_Front = tractDemand * ((1 - front_bias_percent) / front_bias_percent);
    }
    else// rear biased 
    {
        *Outer_Front = tractDemand * (front_bias_percent / (1 - front_bias_percent));
    }

    // torque and momentbal is same sign 
    if ((torque_percent >= 0 && momentBal > 0) || (torque_percent < 0 && momentBal < 0))
    {
        *Inner_Front = momentBal * front_bias_percent;
        *Inner_Rear = momentBal * (1 - front_bias_percent);
    }
    
    else
     {
         *Inner_Front = 0.0;
         *Inner_Rear = 0.0;
     }
}

void seteverything(float torquePer, float mz, float frontbias)
{
    torque_percent = torquePer;
    front_bias_percent = frontbias;
    tractDemand = torquePer * motorPk; 

//SETS MOMENT OF BALANCE EQUATION GIVEN PARAM
    if(front_bias_percent <= 0.5)// rear biased: there is a different momBal equation 
    {
     momentBal =  tractDemand + tractDemand * (front_bias_percent / (1 - front_bias_percent)); 
    }
    else// front biased: different moment bal equation
    {
      momentBal = tractDemand + tractDemand * ((1 - front_bias_percent) /front_bias_percent ); 
    }

//SETS THE SIGN OF MOMENT OF BALANCE EQUATION GIVEN PARM 
    if ((mz > 0.0 && torque_percent >= 0.0) || (mz <= 0.0 && torque_percent < 0.0) )
    {
        momentBal = momentBal + (-1*(mz*alpha));
    }
    else
    {
        momentBal= momentBal + (1*(mz*alpha));
    }
// SETS TORQUE DISTRIBUTION BASED ON 4 CASES 
    if (front_bias_percent <= 0.5) // rear biased 
    {
        if (mz <= 0)//right turn 
        {
            set_torque_distribution(ptr_Tf_RL, ptr_Tf_FL, ptr_Tf_FR, ptr_Tf_RR, 0, tractDemand); 
        }
        else//left turn 
        {
            set_torque_distribution(ptr_Tf_RR, ptr_Tf_FR, ptr_Tf_FL, ptr_Tf_RL, 0, tractDemand); 
        }
    }
    else // frontbiased 
    {
        if (mz <= 0)//right turn 
        {
            set_torque_distribution(ptr_Tf_FL, ptr_Tf_RL, ptr_Tf_FR, ptr_Tf_RR, 1, tractDemand); 
        }
        else//left turn 
        {
            set_torque_distribution(ptr_Tf_FR, ptr_Tf_RR, ptr_Tf_FL, ptr_Tf_RL, 1, tractDemand); 
        }
    }

}

float get_torque(float initalTorque )//checks and corrects overshoot of torque value 
{ 
    
    float torque = 0.0;
    if (initalTorque >= 0.0) // torque is postive, aka acceleration
    {
        if (initalTorque  > motorPk)  // Saturation Check IF Torque exceeds limit
            {
                torque = motorPk;
            }
    }
    else //torque is negative, aka breaking (regen)
        if (initalTorque  > -1 * motorPk) // Saturation Check IF Torque exceeds limit
        {
            torque = -1 * motorPk;
        }
    return torque;
}

int main()
{
    seteverything(0.5, 6.4,0.52);
    printf("Tf_RL: %.4f\n", Tf_RL);
    printf("Tf_FR: %.4f\n", Tf_FR);
    printf("Tf_FL: %.4f\n", Tf_FL);
    printf("Tf_RR: %.4f\n", Tf_RR);
    printf("momentbal: %.4f\n",momentBal);
    printf("trackdemand: %.4f\n",tractDemand);
}