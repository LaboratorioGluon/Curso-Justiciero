
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>

#include <string.h>

#include "libnow.h"

#define MAC_MANDO {0xf0, 0x9e, 0x9e, 0xb5, 0x78, 0xbc}
#define MAC_ROBOT { 0x78,0x21,0x84,0xe0,0xb1,0x94}

uint8_t MACS[2][6] = {MAC_ROBOT, MAC_MANDO};

void libnow_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    // Initialize wifi for ESP-NOW
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize wifi for ESP-NOW
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));

    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    ESP_ERROR_CHECK(esp_now_init());


}


void libnow_addPeer(LibNowDst dst)
{

    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, MACS[dst], 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

void libnow_sendMessage(LibNowDst dst, message_move *msg)
{

    typedef struct 
    {
        uint8_t type;
        message_move msg;
    } __attribute__((packed)) full_message;

    full_message full_msg = {MOVE, *msg};

    esp_now_send(MACS[dst], (uint8_t *)&full_msg, sizeof(full_msg));

    
}

void libnow_sendMessage(LibNowDst dst, message_cal *msg)
{

    typedef struct 
    {
        uint8_t type;
        message_cal msg;
    } __attribute__((packed)) full_message;

    full_message full_msg = {CALIBRATE, *msg};

    esp_now_send(MACS[dst], (uint8_t *)&full_msg, sizeof(full_msg));
}