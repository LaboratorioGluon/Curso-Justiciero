#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <string.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <driver/gpio.h>

#include "libnow.h"

#define MAC_MANDO {0xf0, 0x9e, 0x9e, 0xb5, 0x78, 0xbc}
#define MAC_ROBOT { 0x78,0x21,0x84,0xe0,0xb1,0x94}

extern "C" void app_main();

void app_main() 
{

    gpio_config_t buttonGpio;
    buttonGpio.mode = GPIO_MODE_INPUT;
    buttonGpio.pull_up_en = GPIO_PULLUP_ENABLE;
    buttonGpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    buttonGpio.pin_bit_mask = GPIO_NUM_2;
    buttonGpio.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&buttonGpio);

    /** ADC **/
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));



    libnow_init();
    libnow_addPeer(DST_ROBOT);
    
    uint8_t dataToRobot[10] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0};

    message_move move;
    move.x = 0;
    move.y = 0;
    move.buttons = 0;

    message_cal cal;
    cal.p = 0.0;
    cal.i = 0.0;
    cal.d = 0.0;

    int adcValue = 0;
    for(;;)
    {


        //ESP_LOGI("main", "Message sent");
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adcValue));
        move.y = adcValue;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &adcValue));
        move.x = adcValue;
        //ESP_LOGE("ADC", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adcValue);

        
        move.buttons = gpio_get_level(GPIO_NUM_2);
        ESP_LOGE("MANDO", "X: %d Y: %d, But: %d", move.x, move.y, move.buttons);
        libnow_sendMessage(DST_ROBOT, &move);
        vTaskDelay(pdMS_TO_TICKS(50));
        /*if (move.x%2 )
        {
            cal.d += 0.1f;
            cal.i += 1.0f;
            cal.p += 0.01f;
            libnow_sendMessage(DST_ROBOT, &cal);
        }*/
    }

}