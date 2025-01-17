#include <esp_log.h>
#include <driver/gpio.h>
#include <string.h>

#include "bmi160.h"

static const char *TAG = "BMI160_SPI";

constexpr size_t Length = 8;

void bmi_initSpi(spi_bus_config_t *spiBus, spi_device_handle_t *spiHandle, spi_device_interface_config_t *spiIf, spi_transaction_t *spiTrans)
{
    //
    memset(spiBus, 0, sizeof(spi_bus_config_t));
    memset(spiHandle, 0, sizeof(spi_device_handle_t));
    memset(spiIf, 0, sizeof(spi_device_interface_config_t));
    memset(spiTrans, 0, sizeof(spi_transaction_t));

    // Bus config
    spiBus->mosi_io_num = GPIO_NUM_23;
    spiBus->miso_io_num = GPIO_NUM_19;
    spiBus->sclk_io_num = GPIO_NUM_18;
    spiBus->quadhd_io_num = -1;
    spiBus->quadwp_io_num = -1;
    spiBus->max_transfer_sz = 4092;
    
    //ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busSpi, SPI_DMA_DISABLED));
    if (spi_bus_initialize(SPI3_HOST, spiBus, SPI_DMA_DISABLED) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Bus cannot be initialized");
        return;
    }
    ESP_LOGI(TAG, "SPI Bus Initialized correctly");

    // Interface config
    spiIf->spics_io_num = GPIO_NUM_21; // Cambiame
    spiIf->clock_speed_hz = 1 * 1000 * 1000; // 1Mhz
    spiIf->mode = 0;
    spiIf->queue_size = 10;
    spiIf->address_bits = Length;
    
    //ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiIf, &spiHandle));
    if (spi_bus_add_device(SPI3_HOST, spiIf, spiHandle) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Device cannot be added");
        return;
    }
    ESP_LOGI(TAG, "SPI Device Added correctly");

    // Base address
    uint8_t spiZeroAddr = 0x00;
    uint8_t spiReadAddr = 0x7F;
    uint8_t spiTopAddr = 0x80;
    
    // Transaction config
    uint8_t data;
    spiTrans->length = Length;
    spiTrans->addr = (spiReadAddr | spiTopAddr);
    spiTrans->rxlength = Length;
    spiTrans->rx_buffer = &data;
    //SpiTrans.flags = SPI_TRANS_USE_RXDATA;

    spi_device_acquire_bus(*spiHandle, portMAX_DELAY);
    if (spi_device_polling_transmit(*spiHandle, spiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return;
    }
    ESP_LOGI(TAG, "SPI Transaction OK");
    spi_device_release_bus(*spiHandle);
    ESP_LOGI(TAG, "Received value: %d", data);
    

    spi_device_acquire_bus(*spiHandle, portMAX_DELAY);
    spiTrans->addr = (spiZeroAddr | spiTopAddr);
    if (spi_device_polling_transmit(*spiHandle, spiTrans) != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI Transaction ERROR");
        return;
    }
    ESP_LOGI(TAG, "SPI Transaction OK");
    spi_device_release_bus(*spiHandle);

    ESP_LOGI(TAG, "Received value: %x", data);
}

uint8_t bmi_getId()
{
    return 0;
}