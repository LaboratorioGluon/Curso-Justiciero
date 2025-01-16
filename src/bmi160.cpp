#include "bmi160.h"

#include <esp_log.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>

const char TAG[] = "BMI_DRIVER"; 

void bmi_initSpi()
{
    spi_bus_config_t busSpi = {0};
    spi_device_handle_t SpiHandle = 0;
    spi_device_interface_config_t spiIf = {0};

    busSpi.mosi_io_num = GPIO_NUM_23;
    busSpi.miso_io_num = GPIO_NUM_19;
    busSpi.sclk_io_num = GPIO_NUM_18;
    busSpi.quadhd_io_num = -1;
    busSpi.quadwp_io_num = -1;
    busSpi.max_transfer_sz = 4092;
    
    //ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busSpi, SPI_DMA_DISABLED));
    if ( spi_bus_initialize(SPI3_HOST, &busSpi, SPI_DMA_DISABLED) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Bus cannot be initialized");
        return;
    }
    ESP_LOGE(TAG, "SPI Bus Initialized correctly");

    spiIf.spics_io_num = GPIO_NUM_21; // Cambiame
    spiIf.clock_speed_hz = 1 * 1000 * 1000;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;
    
    //ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiIf, &SpiHandle));
    if ( spi_bus_add_device(SPI3_HOST, &spiIf, &SpiHandle) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Device cannot be added");
        return;
    }
    ESP_LOGE(TAG, "SPI Device Added correctly");

    uint8_t spiReadAddr = 0x7F;
    spi_transaction_t SpiTrans = {0};

    uint8_t data;
    SpiTrans.length = 8;
    SpiTrans.addr = (0x7f | 0x80);
    SpiTrans.rxlength = 8;
    SpiTrans.rx_buffer = &data;
    //SpiTrans.flags = SPI_TRANS_USE_RXDATA;

    spi_device_acquire_bus(SpiHandle, portMAX_DELAY);
    if ( spi_device_polling_transmit(SpiHandle, &SpiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return;
    }
    ESP_LOGE(TAG, "SPI Transaction OK");
    spi_device_release_bus(SpiHandle);
    ESP_LOGE(TAG, "Received value: %d", data);
    

    spi_device_acquire_bus(SpiHandle, portMAX_DELAY);
    SpiTrans.addr = (0x00 | 0x80);
    if ( spi_device_polling_transmit(SpiHandle, &SpiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return;
    }
    ESP_LOGE(TAG, "SPI Transaction OK");
    spi_device_release_bus(SpiHandle);

    ESP_LOGE(TAG, "Received value: %x", data);
}

uint8_t bmi_getId()
{
    return 0;
}