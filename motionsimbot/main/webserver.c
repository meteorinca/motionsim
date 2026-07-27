#include "webserver.h"
#include "config.h"
#include "led.h"
#include "buzzer.h"
#include "timekeep.h"
#include "ota_mgr.h"
#include "oled.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "pid_controller.h"
#include "motion_udp.h"
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
    if (strcmp(action, "estop") == 0) motor_set_estop(true);
    else if (strcmp(action, "resume") == 0) motor_set_estop(false);
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

    motion_stats_t stats = motion_udp_get_stats();
    pid_params_t pid = pid_get_params();

    char resp[768];
    int len = snprintf(resp, sizeof(resp),
        "{"
        "\"m1\":{\"target\":%.2f,\"actual\":%.2f,\"error\":%.2f,\"duty\":%d},"
        "\"m2\":{\"target\":%.2f,\"actual\":%.2f,\"error\":%.2f,\"duty\":%d},"
        "\"m3\":{\"target\":%.2f,\"actual\":%.2f,\"error\":%.2f,\"duty\":%d},"
        "\"pid\":{\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f},"
        "\"estop\":%s,"
        "\"pkt_rate_hz\":%.2f,"
        "\"rx_count\":%ld,"
        "\"lost_count\":%ld,"
        "\"rssi\":%d,"
        "\"ip\":\"%s\","
        "\"version\":\"%s\""
        "}",
        pid_get_target_angle(1), hall_read_angle(1), pid_get_error(1), motor_get_duty(1),
        pid_get_target_angle(2), hall_read_angle(2), pid_get_error(2), motor_get_duty(2),
        pid_get_target_angle(3), hall_read_angle(3), pid_get_error(3), motor_get_duty(3),
        pid.kp, pid.ki, pid.kd,
        motor_get_estop() ? "true" : "false",
        stats.pkt_rate_hz, (long)stats.rx_count, (long)stats.lost_count,
        wifi_mgr_get_rssi(), wifi_mgr_get_ip(), FW_VERSION);

    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

static esp_err_t api_pid_handler(httpd_req_t *req) {
    char buf[128];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        pid_params_t curr = pid_get_params();
        char val[32];
        if (httpd_query_key_value(buf, "kp", val, sizeof(val)) == ESP_OK) curr.kp = atof(val);
        if (httpd_query_key_value(buf, "ki", val, sizeof(val)) == ESP_OK) curr.ki = atof(val);
        if (httpd_query_key_value(buf, "kd", val, sizeof(val)) == ESP_OK) curr.kd = atof(val);
        pid_set_params(curr.kp, curr.ki, curr.kd);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_calibrate_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "joint", val, sizeof(val)) == ESP_OK) {
            int joint = atoi(val);
            hall_calibrate_zero(joint);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_motor_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val_id[16], val_duty[16];
        if (httpd_query_key_value(buf, "id", val_id, sizeof(val_id)) == ESP_OK &&
            httpd_query_key_value(buf, "duty", val_duty, sizeof(val_duty)) == ESP_OK) {
            int id = atoi(val_id);
            int duty = atoi(val_duty);
            motor_set_duty(id, duty);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_estop_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "state", val, sizeof(val)) == ESP_OK) {
            bool state = (atoi(val) != 0);
            motor_set_estop(state);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

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

    static char buf[4096];
    int remaining = total;
    while (remaining > 0) {
        int to_read = remaining < sizeof(buf) ? remaining : sizeof(buf);
        int received = httpd_req_recv(req, buf, to_read);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (received <= 0) {
            ota_abort(&h);
            return ESP_FAIL;
        }
        if (ota_write(&h, buf, received) != ESP_OK) {
            ota_abort(&h);
            return ESP_FAIL;
        }
        remaining -= received;
    }

    if (ota_end(&h) != ESP_OK) return ESP_FAIL;

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
        httpd_uri_t u_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t u_stats = { .uri = "/api/stats", .method = HTTP_GET, .handler = api_stats_handler };
        httpd_uri_t u_pid = { .uri = "/api/pid", .method = HTTP_GET, .handler = api_pid_handler };
        httpd_uri_t u_cal = { .uri = "/api/calibrate", .method = HTTP_POST, .handler = api_calibrate_handler };
        httpd_uri_t u_motor = { .uri = "/api/motor", .method = HTTP_GET, .handler = api_motor_handler };
        httpd_uri_t u_estop = { .uri = "/api/estop", .method = HTTP_POST, .handler = api_estop_handler };
        httpd_uri_t u_ota = { .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler };
        httpd_uri_t u_opt = { .uri = "/*", .method = HTTP_OPTIONS, .handler = cors_options_handler };

        httpd_register_uri_handler(s_server, &u_root);
        httpd_register_uri_handler(s_server, &u_stats);
        httpd_register_uri_handler(s_server, &u_pid);
        httpd_register_uri_handler(s_server, &u_cal);
        httpd_register_uri_handler(s_server, &u_motor);
        httpd_register_uri_handler(s_server, &u_estop);
        httpd_register_uri_handler(s_server, &u_ota);
        httpd_register_uri_handler(s_server, &u_opt);

        ESP_LOGI(TAG, "Webserver started on port %d", config.server_port);
    }
}
