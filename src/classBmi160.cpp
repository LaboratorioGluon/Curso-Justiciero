#include "classBmi160.h"

#include <esp_log.h>

static char * TAG = "BMI160";

constexpr float sensorTimeScale = 0.039f;

/**** PRIVATE FUNCTIONS ****/

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    spi_transaction_t SpiTrans = {0};

    SpiTrans.length = 8*len;
    SpiTrans.addr = reg_addr;
    SpiTrans.rxlength = 8*len;
    SpiTrans.rx_buffer = data;

    spi_device_acquire_bus(Bmi160<Bmi160SpiConfig>::spiHandle, portMAX_DELAY);
    spi_device_polling_transmit(Bmi160<Bmi160SpiConfig>::spiHandle, &SpiTrans);
    spi_device_release_bus(Bmi160<Bmi160SpiConfig>::spiHandle);

    return 0;
}

int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    spi_transaction_t SpiTrans = {0};

    SpiTrans.length = 8*len;
    SpiTrans.addr = reg_addr;
    SpiTrans.tx_buffer = read_data;

    spi_device_acquire_bus(Bmi160<Bmi160SpiConfig>::spiHandle, portMAX_DELAY);
    spi_device_polling_transmit(Bmi160<Bmi160SpiConfig>::spiHandle, &SpiTrans);
    spi_device_release_bus(Bmi160<Bmi160SpiConfig>::spiHandle);

    return 0;
}

void bmi_delay(uint32_t period)
{
    if (period < 10)
    {
        period = 10;
    }
    vTaskDelay(pdMS_TO_TICKS(period));
}

/**** PUBLIC FUNCTIONS ****/


template<>
uint8_t Bmi160<Bmi160SpiConfig>::init(Bmi160SpiConfig config)
{
    spiBus.mosi_io_num = config.mosi;
    spiBus.miso_io_num = config.miso;
    spiBus.sclk_io_num = config.sclk;
    spiBus.quadhd_io_num = -1;
    spiBus.quadwp_io_num = -1;
    spiBus.max_transfer_sz = 4092;
    
    //ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &spiBus, SPI_DMA_DISABLED));
    if ( spi_bus_initialize(config.spidev, &spiBus, SPI_DMA_DISABLED) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Bus cannot be initialized");
        return 1;
    }
    ESP_LOGE(TAG, "SPI Bus Initialized correctly");

    spiIf.spics_io_num = config.cs; 
    spiIf.clock_speed_hz = config.speed;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;
    
    //ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiIf, &SpiHandle));
    if ( spi_bus_add_device(config.spidev, &spiIf, &spiHandle) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Device cannot be added");
        return 2;
    }
    ESP_LOGE(TAG, "SPI Device Added correctly");

    
    dev.intf = BMI160_SPI_INTF;
    dev.read = bmi_read_spi;
    dev.write = bmi_write_spi;
    dev.delay_ms = bmi_delay;


    return configure();
}

template<>
uint8_t Bmi160<Bmi160I2cConfig>::init(Bmi160I2cConfig config)
{

    return configure();
}