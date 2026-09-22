#include "webserver.h"
#include "config.h"
#include "led.h"
#include "buzzer.h"
#include "timekeep.h"
#include "ota_mgr.h"
#include "oled.h"
#include "motor_driver.h"
#include "hall_sensor.h"
#include "as5600.h"
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
    if (strcmp(action, "estop") == 0) pid_trigger_estop(ESTOP_REASON_SOFTWARE, "Software E-Stop triggered");
    else if (strcmp(action, "resume") == 0) pid_clear_estop();
    else if (strcmp(action, "arm") == 0) motor_arm(true);
    else if (strcmp(action, "disarm") == 0) motor_arm(false);
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

// -------------------------------------------------------------
// MyBot-Style Time Endpoints
// -------------------------------------------------------------
static esp_err_t time_handler(httpd_req_t *req) {
    time_t now = timekeep_now();
    bool synced = timekeep_is_synced();
    char resp[128];
    int len = snprintf(resp, sizeof(resp),
        "{\"epoch\":%lld,\"synced\":%s}",
        (long long)now, synced ? "true" : "false");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

static esp_err_t sync_time_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "epoch", param, sizeof(param)) == ESP_OK) {
            time_t t = strtol(param, NULL, 10);
            timekeep_set_time(t);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// -------------------------------------------------------------
// Angle Query and Target Command Endpoints
// -------------------------------------------------------------
// GET /api/angle?joint=1 or GET /api/angles
static esp_err_t api_angle_handler(httpd_req_t *req) {
    char buf[64];
    int joint = 1;
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "joint", val, sizeof(val)) == ESP_OK) {
            joint = atoi(val);
        }
    }
    if (joint < 1 || joint > JOINT_COUNT) joint = 1;

    char resp[256];
    int len = snprintf(resp, sizeof(resp),
        "{\"joint\":%d,\"actual\":%.2f,\"target\":%.2f,\"error\":%.2f,\"duty\":%d,\"raw\":%d}",
        joint,
        pid_get_actual_angle(joint),
        pid_get_target_angle(joint),
        pid_get_error(joint),
        motor_get_duty(joint),
        hall_read_raw(joint));

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

// POST or GET /api/target?joint=1&angle=185.0
static esp_err_t api_target_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val_joint[16], val_angle[32];
        int joint = 1;
        if (httpd_query_key_value(buf, "joint", val_joint, sizeof(val_joint)) == ESP_OK) {
            joint = atoi(val_joint);
        }
        if (httpd_query_key_value(buf, "angle", val_angle, sizeof(val_angle)) == ESP_OK) {
            float angle = atof(val_angle);
            pid_set_mode(PID_MODE_FOLLOW);
            pid_set_target_angle(joint, angle);
            char resp[128];
            snprintf(resp, sizeof(resp), "{\"ok\":true,\"joint\":%d,\"target\":%.2f}", joint, angle);
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, resp);
            return ESP_OK;
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing angle parameter");
    return ESP_FAIL;
}

// -------------------------------------------------------------
// Comprehensive Telemetry & Statistics
// -------------------------------------------------------------
static esp_err_t api_stats_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");

    motion_stats_t stats = motion_udp_get_stats();
    pid_params_t pid = pid_get_params();

    as5600_telemetry_t as_telem = {0};
    as5600_read_telemetry(0, &as_telem);

    char resp[1280];
    int len = snprintf(resp, sizeof(resp),
        "{"
        "\"m1\":{\"target\":%.2f,\"actual\":%.2f,\"error\":%.2f,\"duty\":%d,\"inv\":%s},"
        "\"m2\":{\"target\":%.2f,\"actual\":%.2f,\"error\":%.2f,\"duty\":%d,\"inv\":%s},"
        "\"m3\":{\"target\":%.2f,\"actual\":%.2f,\"error\":%.2f,\"duty\":%d,\"inv\":%s},"
        "\"pid\":{\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f},"
        "\"estop\":%s,"
        "\"estop_code\":%d,"
        "\"estop_reason\":\"%s\","
        "\"auto_estop\":%s,"
        "\"armed\":%s,"
        "\"control_mode\":%d,"
        "\"autotune\":{\"running\":%s,\"status\":\"%s\"},"
        "\"as5600\":{"
            "\"detected\":%s,"
            "\"magnet_ok\":%s,"
            "\"magnet_too_weak\":%s,"
            "\"magnet_too_strong\":%s,"
            "\"agc\":%d,"
            "\"magnitude\":%d,"
            "\"raw\":%d,"
            "\"raw_deg\":%.2f,"
            "\"cal_deg\":%.2f,"
            "\"zero_offset\":%.2f"
        "},"
        "\"time\":{\"epoch\":%lld,\"synced\":%s},"
        "\"pkt_rate_hz\":%.2f,"
        "\"rx_count\":%ld,"
        "\"lost_count\":%ld,"
        "\"rssi\":%d,"
        "\"ip\":\"%s\","
        "\"version\":\"%s\""
        "}",
        pid_get_target_angle(1), pid_get_actual_angle(1), pid_get_error(1), motor_get_duty(1), motor_is_inverted(1) ? "true" : "false",
        pid_get_target_angle(2), pid_get_actual_angle(2), pid_get_error(2), motor_get_duty(2), motor_is_inverted(2) ? "true" : "false",
        pid_get_target_angle(3), pid_get_actual_angle(3), pid_get_error(3), motor_get_duty(3), motor_is_inverted(3) ? "true" : "false",
        pid.kp, pid.ki, pid.kd,
        motor_get_estop() ? "true" : "false",
        (int)pid_get_estop_reason_code(),
        pid_get_estop_reason_str(),
        pid_is_auto_estop_enabled() ? "true" : "false",
        motor_is_armed() ? "true" : "false",
        (int)pid_get_mode(),
        pid_is_autotuning() ? "true" : "false",
        pid_get_autotune_status(),
        as_telem.detected ? "true" : "false",
        as_telem.magnet_detected ? "true" : "false",
        as_telem.magnet_too_weak ? "true" : "false",
        as_telem.magnet_too_strong ? "true" : "false",
        as_telem.agc,
        as_telem.magnitude,
        as_telem.raw_counts,
        as_telem.raw_deg,
        as_telem.cal_deg,
        as_telem.zero_offset,
        (long long)timekeep_now(),
        timekeep_is_synced() ? "true" : "false",
        stats.pkt_rate_hz, (long)stats.rx_count, (long)stats.lost_count,
        wifi_mgr_get_rssi(), wifi_mgr_get_ip(), FW_VERSION);

    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

// -------------------------------------------------------------
// Tuning & Safety Endpoints
// -------------------------------------------------------------
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

static esp_err_t api_pid_save_handler(httpd_req_t *req) {
    esp_err_t err = pid_save_params_to_nvs();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (err == ESP_OK) {
        httpd_resp_sendstr(req, "{\"ok\":true,\"saved\":true}");
    } else {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Save failed\"}");
    }
    return ESP_OK;
}

static esp_err_t api_calibrate_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "joint", val, sizeof(val)) == ESP_OK) {
            int joint = atoi(val);
            hall_calibrate_zero(joint);
            // Re-align target to new 0 position
            pid_set_target_angle(joint, 0.0f);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_arm_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "state", val, sizeof(val)) == ESP_OK) {
            bool state = (atoi(val) != 0);
            motor_arm(state);
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
            if (state) {
                pid_trigger_estop(ESTOP_REASON_SOFTWARE, "Software E-Stop triggered from WebUI");
            } else {
                pid_clear_estop();
            }
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_estop_clear_handler(httpd_req_t *req) {
    pid_clear_estop();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true,\"cleared\":true}");
    return ESP_OK;
}

static esp_err_t api_auto_estop_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "enable", val, sizeof(val)) == ESP_OK) {
            pid_enable_auto_estop(atoi(val) != 0);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// -------------------------------------------------------------
// Commissioning: Motor Jog & Polarity Inversion
// -------------------------------------------------------------
static esp_err_t api_motor_jog_handler(httpd_req_t *req) {
    char buf[64];
    int joint = 1;
    int duty = 0;
    int duration = 500;

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val_j[16], val_d[16], val_t[16];
        if (httpd_query_key_value(buf, "joint", val_j, sizeof(val_j)) == ESP_OK) joint = atoi(val_j);
        if (httpd_query_key_value(buf, "duty", val_d, sizeof(val_d)) == ESP_OK) duty = atoi(val_d);
        if (httpd_query_key_value(buf, "duration", val_t, sizeof(val_t)) == ESP_OK) duration = atoi(val_t);

        motor_jog(joint, (int16_t)duty, (uint32_t)duration);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true,\"jogging\":true}");
    return ESP_OK;
}

static esp_err_t api_motor_invert_handler(httpd_req_t *req) {
    char buf[64];
    int joint = 1;
    bool inv = false;

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val_j[16], val_i[16];
        if (httpd_query_key_value(buf, "joint", val_j, sizeof(val_j)) == ESP_OK) joint = atoi(val_j);
        if (httpd_query_key_value(buf, "inverted", val_i, sizeof(val_i)) == ESP_OK) inv = (atoi(val_i) != 0);

        motor_set_inverted(joint, inv);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t api_autotune_handler(httpd_req_t *req) {
    char buf[64];
    int joint = 1;
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val_j[16];
        if (httpd_query_key_value(buf, "joint", val_j, sizeof(val_j)) == ESP_OK) joint = atoi(val_j);
    }

    esp_err_t err = pid_start_autotune(joint);
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"ok\":%s,\"status\":\"%s\"}",
             err == ESP_OK ? "true" : "false", pid_get_autotune_status());

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// -------------------------------------------------------------
// MyBot-Style Wireless OTA Firmware Update Handler
// -------------------------------------------------------------
#define OTA_BUF_SIZE 4096
static esp_err_t ota_post_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No firmware data");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Incoming OTA firmware binary: %d bytes", total);

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
            ESP_LOGE(TAG, "OTA receive error (%d), aborting", received);
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
        if (((total - remaining) % 65536) < OTA_BUF_SIZE) {
            ESP_LOGI(TAG, "OTA Progress: %d / %d bytes (%.1f%%)",
                     total - remaining, total, ((float)(total - remaining) / (float)total) * 100.0f);
        }
    }

    if (ota_end(&h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA validation failed");
        return ESP_FAIL;
    }

    // Success response before rebooting
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ota\":\"ok\",\"restart\":true}", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "OTA flash success! Rebooting in 1.5 seconds...");
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return ESP_OK;
}

// -------------------------------------------------------------
// Webserver Initialization
// -------------------------------------------------------------
void webserver_start(void) {
    if (s_server) return;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port      = WEB_SERVER_PORT;
    config.stack_size       = 8192;
    config.max_uri_handlers = 32;
    config.recv_wait_timeout= 300; // 300s timeout allows slow WiFi OTA uploads
    config.send_wait_timeout= 10;
    config.lru_purge_enable = true;

    if (httpd_start(&s_server, &config) == ESP_OK) {
        httpd_uri_t uris[] = {
            { "/",                HTTP_GET,     root_get_handler,         NULL },
            { "/time",            HTTP_GET,     time_handler,             NULL },
            { "/sync_time",       HTTP_GET,     sync_time_handler,        NULL },
            { "/api/angle",       HTTP_GET,     api_angle_handler,        NULL },
            { "/api/angles",      HTTP_GET,     api_angle_handler,        NULL },
            { "/api/target",      HTTP_POST,    api_target_handler,       NULL },
            { "/api/target",      HTTP_GET,     api_target_handler,       NULL },
            { "/api/stats",       HTTP_GET,     api_stats_handler,        NULL },
            { "/api/pid",         HTTP_GET,     api_pid_handler,          NULL },
            { "/api/pid/save",    HTTP_POST,    api_pid_save_handler,     NULL },
            { "/api/calibrate",   HTTP_POST,    api_calibrate_handler,    NULL },
            { "/api/arm",         HTTP_POST,    api_arm_handler,          NULL },
            { "/api/estop",       HTTP_POST,    api_estop_handler,        NULL },
            { "/api/estop/clear", HTTP_POST,    api_estop_clear_handler,  NULL },
            { "/api/auto_estop",  HTTP_POST,    api_auto_estop_handler,   NULL },
            { "/api/motor/jog",   HTTP_POST,    api_motor_jog_handler,    NULL },
            { "/api/motor/jog",   HTTP_GET,     api_motor_jog_handler,    NULL },
            { "/api/motor/invert",HTTP_POST,    api_motor_invert_handler, NULL },
            { "/api/autotune",    HTTP_POST,    api_autotune_handler,     NULL },
            { "/api/autotune",    HTTP_GET,     api_autotune_handler,     NULL },
            { "/ota",             HTTP_POST,    ota_post_handler,         NULL },
            { "/ota",             HTTP_OPTIONS, cors_options_handler,     NULL },
            { "/*",               HTTP_OPTIONS, cors_options_handler,     NULL },
        };

        for (int i = 0; i < (int)(sizeof(uris) / sizeof(uris[0])); i++) {
            httpd_register_uri_handler(s_server, &uris[i]);
        }

        ESP_LOGI(TAG, "MotionSimBot HTTP Server started on port %d with %d handlers",
                 config.server_port, (int)(sizeof(uris) / sizeof(uris[0])));
    }
}
