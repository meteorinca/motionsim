#include "i2c_bus.h"
#include "config.h"
#include "esp_log.h"

#define TAG "I2C_BUS"

static i2c_master_bus_handle_t s_bus_handle = NULL;

esp_err_t i2c_bus_init(void) {
    if (s_bus_handle) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus (SDA: %d, SCL: %d): %s",
                 I2C_SDA_PIN, I2C_SCL_PIN, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master bus initialized on SDA=%d, SCL=%d (400 kHz)",
             I2C_SDA_PIN, I2C_SCL_PIN);
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get_handle(void) {
    if (!s_bus_handle) {
        i2c_bus_init();
    }
    return s_bus_handle;
}

esp_err_t i2c_bus_probe(uint16_t address) {
    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (!bus) return ESP_FAIL;
    return i2c_master_probe(bus, address, 100);
}
