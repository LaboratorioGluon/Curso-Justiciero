#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>


class PID{
public:
    PID(float pKp, float pKi, float pKd, float pInt_saturation = 2000);

    float update(float error, float dt);

    void reset();

    void setKp(float pKp){ kp = pKp; }
    void setKi(float pKi){ ki = pKi; }
    void setKd(float pKd){ kd = pKd; }

private:
    float kp, ki, kd;

    float prev_error;
    float prev_d;
    float acum_integral;

    float int_saturation;

    uint8_t first_run;

};


#endif //__PID_H__