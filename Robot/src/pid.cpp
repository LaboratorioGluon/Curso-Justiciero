#include "pid.h"

PID::PID(float pKp, float pKi, float pKd, float pInt_saturation ):
    kp(pKp), ki(pKi), kd(pKd), int_saturation(pInt_saturation)
{
    acum_integral = 0;
    first_run = 1;
    prev_d = 0;
}

float PID::update(float error, float dt)
{

    if(first_run)
    {
        prev_error = error;
        first_run = 0;
    }

    float prop = error;
    
    acum_integral += (error + prev_error)*0.5f*dt;

    if( acum_integral > int_saturation)
    {
        acum_integral = int_saturation;
    }
    else if (acum_integral < -int_saturation)
    {
        acum_integral = -int_saturation;
    }

    float derivative = prev_d*0.5 + ((error - prev_error)/dt)*0.5;
    prev_d = derivative;

    prev_error = error;

    return kp*error + ki*acum_integral + kd*derivative;
}

void PID::reset(void)
{
    first_run = 1;
    ki = 0;
    prev_error = 0;
};