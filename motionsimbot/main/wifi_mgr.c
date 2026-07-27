#include "wifi_mgr.h"
#include "webserver.h"
#include "config.h"
#include "buzzer.h"
#include "oled.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <string.h>
#include <lwip/sockets.h>

#define TAG "WIFI"
#define MAX_NVS_NETWORKS 5
#define AP_FALLBACK_TIMEOUT_MS 10000
#define NVS_WIFI_NAMESPACE "wifi_creds"
#define NVS_KEY_COUNT "count"
#define MAX_TOTAL_NETWORKS (MAX_NVS_NETWORKS + 2)

typedef struct {
    char ssid[33];
    char pass[65];
} wifi_cred_t;

static EventGroupHandle_t s_wifi_events;
static TimerHandle_t s_ap_timer = NULL;
static bool s_ap_active = false;
static int s_no_ap_count = 0;
static int s_auth_fail_count = 0;
static char s_ip_addr[32] = {0};
static wifi_cred_t s_networks[MAX_TOTAL_NETWORKS];
static int s_network_count = 0;
static int s_network_idx = 0;

static void nvs_load_credentials(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    int32_t count = 0;
    nvs_get_i32(h, NVS_KEY_COUNT, &count);
    for (int i = 0; i < count && i < MAX_NVS_NETWORKS; i++) {
        char key_s[32], key_p[32];
        snprintf(key_s, sizeof(key_s), "ssid_%d", i);
        snprintf(key_p, sizeof(key_p), "pass_%d", i);
        size_t slen = sizeof(s_networks[0].ssid);
        size_t plen = sizeof(s_networks[0].pass);
        if (nvs_get_str(h, key_s, s_networks[s_network_count].ssid, &slen) == ESP_OK &&
            nvs_get_str(h, key_p, s_networks[s_network_count].pass, &plen) == ESP_OK) {
            s_network_count++;
        }
    }
    nvs_close(h);
}

esp_err_t wifi_save_credential(const char *ssid, const char *pass) {
    if (!ssid || !pass) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    int32_t count = 0;
    nvs_get_i32(h, NVS_KEY_COUNT, &count);

    for (int i = 0; i < count; i++) {
        char key_s[32], existing_ssid[33] = {0};
        snprintf(key_s, sizeof(key_s), "ssid_%d", i);
        size_t slen = sizeof(existing_ssid);
        if (nvs_get_str(h, key_s, existing_ssid, &slen) == ESP_OK && strcmp(existing_ssid, ssid) == 0) {
            char key_p[32];
            snprintf(key_p, sizeof(key_p), "pass_%d", i);
            nvs_set_str(h, key_p, pass);
            nvs_commit(h);
            nvs_close(h);
            return ESP_OK;
        }
    }

    if (count >= MAX_NVS_NETWORKS) count = MAX_NVS_NETWORKS - 1;
    for (int i = count - 1; i >= 0; i--) {
        char src_s[32], src_p[32], dst_s[32], dst_p[32];
        snprintf(src_s, sizeof(src_s), "ssid_%d", i);
        snprintf(src_p, sizeof(src_p), "pass_%d", i);
        snprintf(dst_s, sizeof(dst_s), "ssid_%d", i + 1);
        snprintf(dst_p, sizeof(dst_p), "pass_%d", i + 1);
        char tmp_s[33] = {0}, tmp_p[65] = {0};
        size_t sl = sizeof(tmp_s), pl = sizeof(tmp_p);
        nvs_get_str(h, src_s, tmp_s, &sl);
        nvs_get_str(h, src_p, tmp_p, &pl);
        nvs_set_str(h, dst_s, tmp_s);
        nvs_set_str(h, dst_p, tmp_p);
    }
    nvs_set_str(h, "ssid_0", ssid);
    nvs_set_str(h, "pass_0", pass);
    nvs_set_i32(h, NVS_KEY_COUNT, count + 1);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

int wifi_nvs_credential_count(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return 0;
    int32_t count = 0;
    nvs_get_i32(h, NVS_KEY_COUNT, &count);
    nvs_close(h);
    return (int)count;
}

bool wifi_nvs_credential_get(int index, char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    int32_t count = 0;
    nvs_get_i32(h, NVS_KEY_COUNT, &count);
    if (index < 0 || index >= count) { nvs_close(h); return false; }
    char key_s[32], key_p[32];
    snprintf(key_s, sizeof(key_s), "ssid_%d", index);
    snprintf(key_p, sizeof(key_p), "pass_%d", index);
    bool ok = (nvs_get_str(h, key_s, ssid, &ssid_len) == ESP_OK && nvs_get_str(h, key_p, pass, &pass_len) == ESP_OK);
    nvs_close(h);
    return ok;
}

esp_err_t wifi_nvs_credential_delete(int index) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    int32_t count = 0;
    nvs_get_i32(h, NVS_KEY_COUNT, &count);
    if (index < 0 || index >= count) { nvs_close(h); return ESP_ERR_INVALID_ARG; }
    for (int i = index; i < count - 1; i++) {
        char src_s[32], src_p[32], dst_s[32], dst_p[32];
        snprintf(src_s, sizeof(src_s), "ssid_%d", i + 1);
        snprintf(src_p, sizeof(src_p), "pass_%d", i + 1);
        snprintf(dst_s, sizeof(dst_s), "ssid_%d", i);
        snprintf(dst_p, sizeof(dst_p), "pass_%d", i);
        char tmp_s[33] = {0}, tmp_p[65] = {0};
        size_t sl = sizeof(tmp_s), pl = sizeof(tmp_p);
        nvs_get_str(h, src_s, tmp_s, &sl);
        nvs_get_str(h, src_p, tmp_p, &pl);
        nvs_set_str(h, dst_s, tmp_s);
        nvs_set_str(h, dst_p, tmp_p);
    }
    char last_s[32], last_p[32];
    snprintf(last_s, sizeof(last_s), "ssid_%d", (int)(count - 1));
    snprintf(last_p, sizeof(last_p), "pass_%d", (int)(count - 1));
    nvs_erase_key(h, last_s);
    nvs_erase_key(h, last_p);
    nvs_set_i32(h, NVS_KEY_COUNT, count - 1);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void wifi_forget_all(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    esp_restart();
}

bool wifi_is_ap_mode(void) { return s_ap_active; }
wifi_state_t wifi_mgr_get_state(void) {
    if (s_ap_active) return WIFI_STATE_AP_MODE;
    if (s_wifi_events && (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT)) return WIFI_STATE_CONNECTED;
    return WIFI_STATE_CONNECTING;
}
const char* wifi_mgr_get_ip(void) { return s_ip_addr; }
int wifi_mgr_get_rssi(void) {
    if (!s_wifi_events || !(xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT)) return 0;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) return ap_info.rssi;
    return 0;
}

static void start_softap(void) {
    if (s_ap_active) return;
    s_ap_active = true;
    wifi_config_t ap_cfg = { .ap = { .ssid_len = 0, .channel = 6, .authmode = WIFI_AUTH_OPEN, .max_connection = 4 } };
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "MotionSimBot-%d", DEVICE_NUMBER);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    webserver_start();
}

static void advance_to_next_network(void) {
    if (s_ap_active) return;
    s_no_ap_count = 0; s_auth_fail_count = 0; s_network_idx++;
    if (s_network_idx < s_network_count) {
        wifi_config_t wifi_config = { .sta = { .threshold.authmode = WIFI_AUTH_WPA_PSK } };
        strncpy((char*)wifi_config.sta.ssid, s_networks[s_network_idx].ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, s_networks[s_network_idx].pass, sizeof(wifi_config.sta.password));
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_connect();
        if (s_ap_timer) xTimerReset(s_ap_timer, 0);
    } else {
        start_softap();
    }
}

static void ap_fallback_cb(TimerHandle_t xTimer) { advance_to_next_network(); }

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    if (id == WIFI_EVENT_STA_START) {
        if (s_network_count == 0) advance_to_next_network();
        else { esp_wifi_connect(); if (s_ap_timer) xTimerReset(s_ap_timer, 0); }
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        if (!s_ap_active) esp_wifi_connect();
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip_addr, sizeof(s_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
        if (s_ap_timer) xTimerStop(s_ap_timer, 0);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
        buzzer_demo_wifi_connected();
        webserver_start();
    }
}

static void build_network_list(void) {
    s_network_count = 0; nvs_load_credentials();
    const char *ssids[] = { WIFI_SSID_1, WIFI_SSID_2 };
    const char *passes[] = { WIFI_PASS_1, WIFI_PASS_2 };
    for (int h = 0; h < 2 && s_network_count < MAX_TOTAL_NETWORKS; h++) {
        if (ssids[h][0] == '\0') continue;
        strncpy(s_networks[s_network_count].ssid, ssids[h], sizeof(s_networks[0].ssid) - 1);
        strncpy(s_networks[s_network_count].pass, passes[h], sizeof(s_networks[0].pass) - 1);
        s_network_count++;
    }
}

EventGroupHandle_t wifi_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event, NULL, NULL);
    build_network_list();
    wifi_config_t wifi_config = { .sta = { .threshold.authmode = WIFI_AUTH_WPA2_PSK } };
    if (s_network_count > 0) {
        strncpy((char*)wifi_config.sta.ssid, s_networks[0].ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, s_networks[0].pass, sizeof(wifi_config.sta.password));
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Ultra-low latency mode
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(MDNS_HOSTNAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(MDNS_INSTANCE));
    mdns_service_add(MDNS_INSTANCE, "_http", "_tcp", WEB_SERVER_PORT, NULL, 0);
    s_ap_timer = xTimerCreate("ap_fb", pdMS_TO_TICKS(AP_FALLBACK_TIMEOUT_MS), pdFALSE, NULL, ap_fallback_cb);
    return s_wifi_events;
}
