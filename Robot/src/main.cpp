
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
//PID pid(250.0, 100.0, 2.0f);
// Bastante bien: PID pid(100.0, 300.0, 3.0f);
PID pid(100.0f, 100.0f, 0.3f);

//#define deltaAlphaRange 32.0f
#define deltaAlphaRange 0.0f
#define turnRange 300
float deltaAlpha = 0.0f;
float turn = 0.0f;

constexpr gpio_num_t AIN1 = GPIO_NUM_25;
constexpr gpio_num_t AIN2 = GPIO_NUM_33;
constexpr gpio_num_t BIN1 = GPIO_NUM_26;
constexpr gpio_num_t BIN2 = GPIO_NUM_27;

motorController motors(AIN1, AIN2, GPIO_NUM_32, BIN1, BIN2, GPIO_NUM_14);

void doWhenMove(message_move msg)
{
    //ESP_LOGE("MAIN", "Message Move");
    //ESP_LOGE("MAIN", "X: %d, Y: %d, Buttons: %d", msg.x, msg.y, msg.buttons);
    deltaAlpha = (msg.x-2270)/4095.0f;
    turn = (msg.y-2317)/4095.0f;
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
    float factor = 0.985f;
    //float factor = 1.0f;
    Bmi160::Data acc, gyr;
    float lastTime = 0.0f;
    float dt;
    int32_t motor, prev_motor = 0;

    imu.getData(acc, gyr);
    lastTime = gyr.time;
    alpha = atan2f(acc.z+0.06, acc.y+0.11)*180.0f/M_PI;

    ESP_LOGE("MAIN", "Acc: %.2f %.2f %.2f", acc.x, acc.y, acc.z);
    ESP_LOGE("MAIN", "Starting alpha: %.2f", alpha);
    float p,i,d;
    for(;;)
    {
        // Step 1: Calculate alpha ( Angle from the vertical )
        imu.getData(acc, gyr);
        
        //printf("Acc: %.2f %.2f %.2f\n", acc.x, acc.y, acc.z);
        //ESP_LOGE("MAIN", "Gyr: %.2f %.2f %.2f", gyr.x, gyr.y, gyr.z);
        dt = (gyr.time - lastTime)/1000.0f;
        lastTime = gyr.time;
        

        // acc.y look for the angle that it is zero when the robot is standing
        // and varies when the motor falls.

        alpha = (alpha - gyr.x*dt ) * factor + atan2f(acc.z+0.06, acc.y+0.11)*180.0f/M_PI * (1.0f-factor);

        // Sleep mode
        if ((alpha > -45.0f) || (alpha < -90-45) )
        {
            motors.setSpeed(0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        //alpha=  (alpha - gyr.x*dt ) * factor + acc.y*180.0f/M_PI * (1.0f-factor);
        //ESP_LOGE("MAIN", "Alpha: %.2f %.2f %.2f", alpha, gyr.x*dt, atan2f(acc.z, acc.y+0.16)*180.0f/M_PI);
        //printf("$%.2f;\n", alpha);
        // Step 2: Calculat.0, 0.0, 0.0);e motors using PID
        motor = pid.update((-89.0f-alpha-deltaAlpha*deltaAlphaRange), dt);
        //motor = pid.update((-7-alpha-deltaAlpha*deltaAlphaRange), dt);
        pid.getLastPid(p,i,d);
        //ESP_LOGE("MAIN", "PID: %.4f: %.3f, %.3f, %.3f",dt, p,i,d);
        
        if (motor > 0)
        {
            motor += 100;
        }
        if (motor < 0)
        {
            motor -= 100;
        }

        motor = prev_motor*0.1 + motor*0.9;
        prev_motor = motor;
        motors.setSpeed(-motor+turn*turnRange, -motor-turn*turnRange);
        //ESP_LOGE("MAIN", "Motor: %ld", motor);
        printf("$%.2f, %ld, %.3f, %.3f, %.3f;\n", alpha, motor,p,i,d);
        vTaskDelay(2);
    }

}