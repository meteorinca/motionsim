#ifndef WIFI_MGR_H
#define WIFI_MGR_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <stdbool.h>

#define WIFI_CONNECTED_BIT BIT0

typedef enum {
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_AP_MODE
} wifi_state_t;

EventGroupHandle_t wifi_init(void);
wifi_state_t wifi_mgr_get_state(void);
const char* wifi_mgr_get_ip(void);
int wifi_mgr_get_rssi(void);
bool wifi_is_ap_mode(void);

esp_err_t wifi_save_credential(const char *ssid, const char *pass);
int wifi_nvs_credential_count(void);
bool wifi_nvs_credential_get(int index, char *ssid, size_t ssid_len, char *pass, size_t pass_len);
esp_err_t wifi_nvs_credential_delete(int index);
void wifi_forget_all(void);

#endif
