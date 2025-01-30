
//#include "localBmi160.h"
#include "classBmi160.h"

#include <freertos/FreeRTOS.h>


extern "C" void app_main();


Bmi160<Bmi160SpiConfig> imu;

void app_main() {

    vTaskDelay(pdMS_TO_TICKS(1000));

    Bmi160SpiConfig config = {
        .spidev = SPI3_HOST,
        .mosi = GPIO_NUM_23,
        .miso = GPIO_NUM_19,
        .sclk = GPIO_NUM_18,
        .cs   = GPIO_NUM_21,
        .speed= 4*1000*1000
    };

    imu.init(config);

    bmi160_sensor_data acc, gyr;

    for(;;)
    {
        imu.getRawData(acc, gyr);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

#if 0
    //bmi_initSpi();
    dev = bmi_init_spi();

    int8_t rslt = bmi160_init(&dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
    }

    /* Select the Output data rate, range of accelerometer sensor */
    dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;

    /* Select the power mode of accelerometer sensor */
    dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    float accTog = 16.0f / float(( (1<< 15) - 1 ));
    float gyrTog = 2000.0f / float(( (1<< 15) - 1 ));

    /* Select the power mode of Gyroscope sensor */
    dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&dev);

    bmi160_sensor_data acc;
    bmi160_sensor_data gyr;


    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if ( bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &acc, &gyr, &dev) == BMI160_OK)
        {
            printf("Acc: %6.3f %6.3f %6.3f %.3f\n", acc.x*accTog, acc.y*accTog, acc.z*accTog, acc.sensortime*0.039f);
            //printf("Gyr: %6d %6d %6d\n", gyr.x, gyr.y, gyr.z);
        }
    }

#endif
}