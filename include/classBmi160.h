#ifndef CLASSBMI160_H__
#define CLASSBMI160_H__

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include "bmi160.h"

#include <esp_log.h>

typedef struct {
    spi_host_device_t spidev;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    uint32_t speed;
} Bmi160SpiConfig;


typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t addr;
} Bmi160I2cConfig;

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi_delay(uint32_t period);

template<typename T>
class Bmi160{
public:

    typedef struct{
        float x;
        float y;
        float z;
        float time;
    } Data;

    Bmi160();

    uint8_t init(T config);
    /*uint8_t init(Bmi160SpiConfig spiConfig);
    uint8_t init(Bmi160I2cConfig i2cConfig);*/

    uint8_t configure();

    uint8_t getRawData(bmi160_sensor_data &acc, bmi160_sensor_data &gyr);
    uint8_t getData(Data &acc, Data &gyr);

    static spi_device_handle_t spiHandle;

private:

    bmi160_dev dev;
    float accScale;
    float gyrScale;
    static constexpr char * TAG = "BMI160";

    float sensorTimeScale = 0.039f;
    
    // SPI
    spi_bus_config_t spiBus;
    spi_device_interface_config_t spiIf;

};



template<typename T>
spi_device_handle_t Bmi160<T>::spiHandle = {0};

template<typename T>
Bmi160<T>::Bmi160():
     dev({0}), spiBus({0}), spiIf({0})
{
    
}


template<typename T>
uint8_t Bmi160<T>::configure()
{
    int8_t rslt = bmi160_init(&dev);

    if (rslt == BMI160_OK)
    {
        ESP_LOGE(TAG,"BMI160 initialization success !\n");
        ESP_LOGE(TAG,"Chip ID 0x%X\n", dev.chip_id);
    }
    else
    {
        ESP_LOGE(TAG,"BMI160 initialization failure !\n");
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

    accScale = 16.0f / float(( (1<< 15) - 1 ));
    gyrScale = 2000.0f / float(( (1<< 15) - 1 ));

    /* Select the power mode of Gyroscope sensor */
    dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&dev);

    if( rslt != BMI160_OK)
    {
        ESP_LOGE(TAG, "Sensor configuration failed");
    }

    return rslt;
}

template<typename T>
uint8_t Bmi160<T>::getRawData(bmi160_sensor_data &acc, bmi160_sensor_data &gyr)
{
    uint8_t ret = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &acc, &gyr, &dev);
    ESP_LOGE(TAG, "Acc: %6d %6d %6d %6lu\n", acc.x, acc.y, acc.z, acc.sensortime);
    return ret;
}

template<typename T>
uint8_t Bmi160<T>::getData(Data &acc, Data &gyr)
{
    bmi160_sensor_data rawAcc, rawGyr;
    uint8_t ret = getRawData(rawAcc, rawGyr);

    acc.x = rawAcc.x*accScale;
    acc.y = rawAcc.y*accScale;
    acc.z = rawAcc.z*accScale;
    acc.time = rawAcc.sensortime * sensorTimeScale;

    gyr.x = rawGyr.x*gyrScale;
    gyr.y = rawGyr.y*gyrScale;
    gyr.z = rawGyr.z*gyrScale;
    gyr.time = rawGyr.sensortime * sensorTimeScale;

    return ret;
}


#endif