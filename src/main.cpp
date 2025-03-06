
//#include "localBmi160.h"
#include "classBmi160.h"
#include "servo.h"
#include "pid.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#include "motorController.h"

extern "C" void app_main();


Bmi160 imu;
Servo arm;
PID pid(300.0, 0.0, 0.0);

constexpr gpio_num_t AIN1 = GPIO_NUM_25;
constexpr gpio_num_t AIN2 = GPIO_NUM_33;
constexpr gpio_num_t BIN1 = GPIO_NUM_26;
constexpr gpio_num_t BIN2 = GPIO_NUM_27;

motorController motors(AIN1, AIN2, GPIO_NUM_32, BIN1, BIN2, GPIO_NUM_14);

void app_main() {

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

    float alpha = 0.0f; // [degree]
    float factor = 0.995f;
    Bmi160::Data acc, gyr;
    float lastTime = 0.0f;
    float dt;
    int32_t motor;

    
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
        alpha = (alpha + gyr.x*dt ) * factor + acc.y*9.8f * (1-factor);
        //ESP_LOGE("MAIN", "Alpha: %.2f", alpha);

        // Step 2: Calculat.0, 0.0, 0.0);e motors using PID
        motor = pid.update(-alpha, dt);
        motors.setSpeed(motor, motor);
        ESP_LOGE("MAIN", "Motor: %ld", motor);
        vTaskDelay(2);
    }

}