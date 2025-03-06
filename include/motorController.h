#ifndef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__

#include <driver/gpio.h>
#include <driver/ledc.h>


class motorController{
public:
    motorController( gpio_num_t leftA, gpio_num_t leftB, gpio_num_t leftPwm ,
                     gpio_num_t rightA, gpio_num_t rightB, gpio_num_t rightPwm);

    /**
     * @brief Initialize the hardware modules.
     */
    void init();

    /**
     * @brief Set each motor speed. The values can be negative ( wheel going backwards ).
     *        Both parameters shall be between 0 - 1024
     * 
     * @param leftSpeed 
     * @param rightSpeed 
     */
    void setSpeed( int32_t leftSpeed, int32_t rightSpeed);

private:

    gpio_num_t leftPinA, leftPinB, leftPinPwm;
    gpio_num_t rightPinA, rightPinB, rightPinPwm;

    ledc_channel_t leftPwmChannel, rightPwmChannel;

    typedef enum{
        DIR_FWD = 0,
        DIR_BCK
    }Direction;

    Direction dirL, dirR; 



};

#endif //__MOTORCONTROLLER_H__