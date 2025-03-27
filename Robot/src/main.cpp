
//#include "localBmi160.h"
#include "classBmi160.h"
#include "servo.h"
#include "pid.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#include "motorController.h"

#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <esp_mac.h>

#include "libnow.h"

#define MAC_MANDO { 0xf0, 0x9e, 0x9e, 0xb5, 0x78, 0xbc}
#define MAC_ROBOT { 0x78, 0x21, 0x84, 0xe0, 0xb1, 0x94}


extern "C" void app_main();


Bmi160 imu;
Servo arm;
PID pid(200.0, 0.5, 0.01);

constexpr gpio_num_t AIN1 = GPIO_NUM_25;
constexpr gpio_num_t AIN2 = GPIO_NUM_33;
constexpr gpio_num_t BIN1 = GPIO_NUM_26;
constexpr gpio_num_t BIN2 = GPIO_NUM_27;

motorController motors(AIN1, AIN2, GPIO_NUM_32, BIN1, BIN2, GPIO_NUM_14);

void doWhenMove(message_move msg)
{
    ESP_LOGE("MAIN", "Message Move");
    ESP_LOGE("MAIN", "X: %d, Y: %d, Buttons: %d", msg.x, msg.y, msg.buttons);
}

void doWhenCal(message_cal msg)
{
    ESP_LOGE("MAIN", "Message Cal");
    ESP_LOGE("MAIN", "P: %f, I: %f, D: %f", msg.p, msg.i, msg.d);
}

static void recvcb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
    ESP_LOGE("MAIN", "Message received");

    if ( (data[0] == MOVE) )
    {
        doWhenMove(*(message_move*)&data[1]);
    }
    else if ( (data[0] == CALIBRATE) )
    {
        doWhenCal(*(message_cal*)&data[1]);
    }
}

void app_main() {

    vTaskDelay(pdMS_TO_TICKS(2000));
    libnow_init();

    esp_now_register_recv_cb(recvcb);

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGE("MAIN", "Estoy vivo");
    }
    // Peripheral init
    Bmi160SpiConfig config = {
        .spidev = SPI3_HOST,
        .mosi = GPIO_NUM_23,
        .miso = GPIO_NUM_19,
        .sclk = GPIO_NUM_18,
        .cs   = GPIO_NUM_21,
        .speed= 4*1000*1000
    };

    imu.init(config);
    motors.init();

    ESP_LOGE("MAIN", "Calibrating");
    imu.calibrate(3000);
    ESP_LOGE("MAIN", "Calibrated");

    float alpha = 0.0f; // [degree]
    float factor = 0.995f;
    //float factor = 1.0f;
    Bmi160::Data acc, gyr;
    float lastTime = 0.0f;
    float dt;
    int32_t motor, prev_motor = 0;

    imu.getData(acc, gyr);
    lastTime = gyr.time;
    alpha = atan2f(acc.y, acc.z)*180.0f/M_PI_2;

    uint32_t speed = 60;
    for(;;)
    {
        motors.setSpeed(speed, speed);
        speed += 10;
        ESP_LOGE("BLACAI", "Motores al %ld", speed);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    for(;;)
    {
        // Step 1: Calculate alpha ( Angle from the vertical )
        imu.getData(acc, gyr);
        //ESP_LOGE("MAIN", "Acc: %.2f %.2f %.2f", acc.x, acc.y, acc.z);
        //ESP_LOGE("MAIN", "Gyr: %.2f %.2f %.2f", gyr.x, gyr.y, gyr.z);
        dt = (gyr.time - lastTime)/1000.0f;
        lastTime = gyr.time;
        

        // acc.y look for the angle that it is zero when the robot is standing
        // and varies when the motor falls.
        alpha = (alpha + gyr.x*dt ) * factor + atan2f(acc.y, acc.z)*180.0f/M_PI_2 * (1-factor);
        //ESP_LOGE("MAIN", "Alpha: %.2f %.2f", alpha, atan2f(acc.y, acc.z)*180.0f/M_PI_2);

        // Step 2: Calculat.0, 0.0, 0.0);e motors using PID
        motor = pid.update((-4.0f-alpha), dt);
        
        if (motor > 0)
        {
            motor += 100;
        }
        if (motor < 0)
        {
            motor -= 100;
        }

        motor = (motor > 1023)?1023:motor;
        motor = (motor < -1023)?-1023:motor;

        motor = prev_motor*0.1 + motor*0.9;
        prev_motor = motor;
        motors.setSpeed(motor, motor);
        //ESP_LOGE("MAIN", "Motor: %ld", motor);
        printf("$%.2f %ld;\n", alpha, motor);
        vTaskDelay(2);
    }

}