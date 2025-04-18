#include "motorController.h"
#include <string.h>

motorController::motorController(gpio_num_t leftA, gpio_num_t leftB, gpio_num_t leftPwm,
                                 gpio_num_t rightA, gpio_num_t rightB, gpio_num_t rightPwm)
{
    leftPinA = leftA;
    leftPinB = leftB;
    leftPinPwm = leftPwm;
    rightPinA = rightA;
    rightPinB = rightB;
    rightPinPwm = rightPwm;
    leftPwmChannel = LEDC_CHANNEL_2;
    rightPwmChannel = LEDC_CHANNEL_3;
}

void motorController::init()
{
    ledc_timer_config_t ledcTimer;
    ledcTimer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledcTimer.freq_hz = 200;
    ledcTimer.duty_resolution = LEDC_TIMER_10_BIT;
    ledcTimer.clk_cfg = LEDC_AUTO_CLK;
    ledcTimer.timer_num = LEDC_TIMER_1;
    ledcTimer.deconfigure = 0;
    ESP_ERROR_CHECK(ledc_timer_config(&ledcTimer));

    ledc_channel_config_t ledcChannel;
    memset(&ledcChannel, 0, sizeof(ledcChannel));
    ledcChannel.channel = leftPwmChannel;
    ledcChannel.gpio_num = leftPinPwm;
    ledcChannel.duty = 0;
    ledcChannel.timer_sel = LEDC_TIMER_1;
    ledcChannel.intr_type = LEDC_INTR_DISABLE;
    ledcChannel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledcChannel.flags.output_invert = 0;
    ledcChannel.hpoint = 0;
    
    ESP_ERROR_CHECK(ledc_channel_config(&ledcChannel));

    ledcChannel.channel = rightPwmChannel;
    ledcChannel.gpio_num = rightPinPwm;
    ESP_ERROR_CHECK(ledc_channel_config(&ledcChannel));

    gpio_config_t config;
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask = (((uint64_t)1) << leftPinA) |  (((uint64_t)1) << leftPinB) |  (((uint64_t)1) << rightPinA)|  (((uint64_t)1) << rightPinB);
    config.intr_type = GPIO_INTR_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&config);

    gpio_set_level(leftPinA, 0);
    gpio_set_level(leftPinB, 1);
    gpio_set_level(rightPinA, 0);
    gpio_set_level(rightPinB, 1);

    dirL = DIR_FWD;
    dirR = DIR_FWD;

}

void motorController::setSpeed(int32_t leftSpeed, int32_t rightSpeed)
{

    if( leftSpeed > 0 && dirL == DIR_BCK)
    {
        gpio_set_level(leftPinA, 0);
        gpio_set_level(leftPinB, 1);
        dirL = DIR_FWD;
    }
    else if( leftSpeed < 0 && dirL == DIR_FWD)
    {
        gpio_set_level(leftPinA, 1);
        gpio_set_level(leftPinB, 0);
        dirL = DIR_BCK;
    }

    if( rightSpeed > 0 && dirR == DIR_BCK)
    {
        gpio_set_level(rightPinA, 0);
        gpio_set_level(rightPinB, 1);
        dirR = DIR_FWD;
    }
    else if( rightSpeed < 0 && dirR == DIR_FWD)
    {
        gpio_set_level(rightPinA, 1);
        gpio_set_level(rightPinB, 0);
        dirR = DIR_BCK;
    }

    //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 300+(int)result);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, leftPwmChannel, abs(leftSpeed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, leftPwmChannel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, rightPwmChannel, abs(rightSpeed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, rightPwmChannel);
}
