#include <freertos/FreeRTOS.h>

#include "bmi160.h"

extern "C" void app_main();

// SPI structs
spi_bus_config_t spiBus;
spi_device_handle_t spiHandle;
spi_device_interface_config_t spiIf;
spi_transaction_t spiTrans;

void app_main() {

    vTaskDelay(pdMS_TO_TICKS(1000));

    bmi_initSpi(&spiBus, &spiHandle, &spiIf, &spiTrans);

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}