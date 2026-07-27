#include "webserver.h"
#include "config.h"
#include "led.h"
#include "buzzer.h"
#include "timekeep.h"
#include "ota_mgr.h"
#include "oled.h"
#include "bench_udp.h"
#include "wifi_mgr.h"

#include <sys/param.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "WEBSERVER"

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static httpd_handle_t s_server = NULL;

void execute_named_action(const char *action) {
    ESP_LOGI("ACTION", "Executing: %s", action);
    if      (strcmp(action, "l1on")   == 0) led_action_set(true);
    else if (strcmp(action, "l1off")  == 0) led_action_set(false);
    else if (strcmp(action, "toggle") == 0) led_action_toggle();
    else if (strcmp(action, "grnon")  == 0) led_grn_set(true);
    else if (strcmp(action, "grnoff") == 0) led_grn_set(false);
    else if (strcmp(action, "redon")  == 0) led_red_set(true);
    else if (strcmp(action, "redoff") == 0) led_red_set(false);
    else if (strcmp(action, "oled_next") == 0) oled_cycle_mode();
    else ESP_LOGW("ACTION", "Action unknown: %s", action);
}

static esp_err_t cors_options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS, DELETE");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t api_stats_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");

    bench_stats_t stats = bench_udp_get_stats();

    char resp[512];
    int len = snprintf(resp, sizeof(resp),
        "{"
        "\"rx_count\":%ld,"
        "\"tx_count\":%ld,"
        "\"lost_count\":%ld,"
        "\"dup_count\":%ld,"
        "\"out_of_order\":%ld,"
        "\"safety_trips\":%ld,"
        "\"max_rx_interval_us\":%ld,"
        "\"pkt_rate_hz\":%.2f,"
        "\"rssi\":%d,"
        "\"safety_stopped\":%s,"
        "\"ip\":\"%s\","
        "\"oled_mode\":%d,"
        "\"version\":\"%s\""
        "}",
        (long)stats.rx_count,
        (long)stats.tx_count,
        (long)stats.lost_count,
        (long)stats.dup_count,
        (long)stats.out_of_order_count,
        (long)stats.safety_trips,
        (long)stats.max_rx_interval_us,
        stats.pkt_rate_hz,
        stats.rssi,
        stats.safety_stopped ? "true" : "false",
        wifi_mgr_get_ip(),
        (int)oled_get_mode(),
        FW_VERSION);

    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

static esp_err_t api_reset_handler(httpd_req_t *req) {
    bench_udp_reset_stats();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_oled_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char mode_str[32];
        if (httpd_query_key_value(buf, "mode", mode_str, sizeof(mode_str)) == ESP_OK) {
            if (strcmp(mode_str, "stats") == 0) oled_set_mode(OLED_MODE_BENCH_STATS);
            else if (strcmp(mode_str, "graph") == 0) oled_set_mode(OLED_MODE_BENCH_GRAPH);
            else if (strcmp(mode_str, "dr2") == 0) oled_set_mode(OLED_MODE_DR2_TELEMETRY);
            else if (strcmp(mode_str, "wifi") == 0) oled_set_mode(OLED_MODE_WIFI_INFO);
            else if (strcmp(mode_str, "eyes") == 0) oled_set_mode(OLED_MODE_EYES);
            else if (strcmp(mode_str, "cycle") == 0) oled_cycle_mode();
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_reboot_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"reboot\":true}");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    char resp[128];
    int len = snprintf(resp, sizeof(resp),
        "{\"status\":\"running\",\"version\":\"%s\",\"ip\":\"%s\"}",
        FW_VERSION, wifi_mgr_get_ip());
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

#define OTA_BUF_SIZE 4096
static esp_err_t ota_post_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No firmware data");
        return ESP_FAIL;
    }

    ota_handle_t h = {0};
    if (ota_begin(&h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    static char buf[OTA_BUF_SIZE];
    int remaining = total;
    while (remaining > 0) {
        int to_read = remaining < OTA_BUF_SIZE ? remaining : OTA_BUF_SIZE;
        int received = httpd_req_recv(req, buf, to_read);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (received <= 0) {
            ota_abort(&h);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }
        if (ota_write(&h, buf, received) != ESP_OK) {
            ota_abort(&h);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write error");
            return ESP_FAIL;
        }
        remaining -= received;
    }

    if (ota_end(&h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA validation failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ota\":\"ok\",\"restart\":true}", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return ESP_OK;
}

void webserver_start(void) {
    if (s_server) return;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096;
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true;

    if (httpd_start(&s_server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_stats = { .uri = "/api/stats", .method = HTTP_GET, .handler = api_stats_handler };
        httpd_uri_t uri_reset = { .uri = "/api/reset", .method = HTTP_GET, .handler = api_reset_handler };
        httpd_uri_t uri_oled = { .uri = "/api/oled", .method = HTTP_GET, .handler = api_oled_handler };
        httpd_uri_t uri_reboot = { .uri = "/api/reboot", .method = HTTP_GET, .handler = api_reboot_handler };
        httpd_uri_t uri_status = { .uri = "/status", .method = HTTP_GET, .handler = status_handler };
        httpd_uri_t uri_ota = { .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler };
        httpd_uri_t uri_options = { .uri = "/*", .method = HTTP_OPTIONS, .handler = cors_options_handler };

        httpd_register_uri_handler(s_server, &uri_root);
        httpd_register_uri_handler(s_server, &uri_stats);
        httpd_register_uri_handler(s_server, &uri_reset);
        httpd_register_uri_handler(s_server, &uri_oled);
        httpd_register_uri_handler(s_server, &uri_reboot);
        httpd_register_uri_handler(s_server, &uri_status);
        httpd_register_uri_handler(s_server, &uri_ota);
        httpd_register_uri_handler(s_server, &uri_options);

        ESP_LOGI(TAG, "Webserver started on port %d", config.server_port);
    }
}
