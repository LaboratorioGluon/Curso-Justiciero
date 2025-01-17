#ifndef __BMI160_H__
#define __BMI160_H__

#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include <stdint.h>

void bmi_initSpi(spi_bus_config_t *spiBus, spi_device_handle_t *spiHandle, spi_device_interface_config_t *spiIf, spi_transaction_t *spiTrans);

uint8_t bmi_getId();

#endif //__BMI160_H__