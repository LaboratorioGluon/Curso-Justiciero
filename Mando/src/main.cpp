#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <string.h>

#include "libnow.h"

#define MAC_MANDO {0xf0, 0x9e, 0x9e, 0xb5, 0x78, 0xbc}
#define MAC_ROBOT { 0x78,0x21,0x84,0xe0,0xb1,0x94}

extern "C" void app_main();

void app_main() 
{
    vTaskDelay(pdMS_TO_TICKS(2000));
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

    for(;;)
    {

        ESP_LOGI("main", "Hello, world! v2");
        libnow_sendMessage(DST_ROBOT, &move);
        //comms.sendRawData(dataToRobot, 10);
        ESP_LOGI("main", "Message sent");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        move.x += 1;
        move.y += 3;
        move.buttons = !move.buttons;


        if (move.x%2 )
        {
            cal.d += 0.1f;
            cal.i += 1.0f;
            cal.p += 0.01f;
            libnow_sendMessage(DST_ROBOT, &cal);
        }
    }

}