#include "ota_mgr.h"
#include "esp_log.h"
#include "esp_system.h"

volatile ota_state_t g_ota_state = OTA_STATE_IDLE;

esp_err_t ota_begin(ota_handle_t *h) {
    if (!h) return ESP_ERR_INVALID_ARG;
    h->partition = esp_ota_get_next_update_partition(NULL);
    if (!h->partition) return ESP_ERR_NOT_FOUND;
    esp_err_t err = esp_ota_begin(h->partition, OTA_WITH_SEQUENTIAL_WRITES, &h->handle);
    if (err == ESP_OK) g_ota_state = OTA_STATE_ACTIVE;
    return err;
}

esp_err_t ota_write(ota_handle_t *h, const void *data, size_t len) {
    if (!h || !data || len == 0) return ESP_ERR_INVALID_ARG;
    return esp_ota_write(h->handle, data, len);
}

esp_err_t ota_end(ota_handle_t *h) {
    if (!h) return ESP_ERR_INVALID_ARG;
    esp_err_t err = esp_ota_end(h->handle);
    if (err != ESP_OK) { g_ota_state = OTA_STATE_FAILED; return err; }
    err = esp_ota_set_boot_partition(h->partition);
    if (err == ESP_OK) g_ota_state = OTA_STATE_SUCCESS;
    return err;
}

void ota_abort(ota_handle_t *h) {
    if (h && h->handle) esp_ota_abort(h->handle);
    g_ota_state = OTA_STATE_FAILED;
}
