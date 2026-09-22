#include "i2c_bus.h"
#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define TAG "I2C_BUS"

static i2c_master_bus_handle_t s_bus_handle = NULL;

esp_err_t i2c_bus_init(void) {
    if (s_bus_handle) {
        return ESP_OK;
    }

    // 1. Enable internal pull-ups on SDA and SCL before bus creation
    gpio_set_pull_mode(I2C_SDA_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(I2C_SCL_PIN, GPIO_PULLUP_ONLY);

    int sda_lvl = gpio_get_level(I2C_SDA_PIN);
    int scl_lvl = gpio_get_level(I2C_SCL_PIN);
    ESP_LOGI(TAG, "I2C pin pre-check: SDA(GPIO %d)=%d, SCL(GPIO %d)=%d",
             I2C_SDA_PIN, sda_lvl, I2C_SCL_PIN, scl_lvl);

    // If SDA is held low by a slave waiting for clock cycles, pulse SCL 9 times
    if (sda_lvl == 0) {
        ESP_LOGW(TAG, "SDA line is LOW! Toggling SCL 9 times to release any stuck I2C slave...");
        gpio_set_direction(I2C_SCL_PIN, GPIO_MODE_OUTPUT);
        for (int i = 0; i < 9; i++) {
            gpio_set_level(I2C_SCL_PIN, 0);
            esp_rom_delay_us(10);
            gpio_set_level(I2C_SCL_PIN, 1);
            esp_rom_delay_us(10);
        }
        gpio_set_direction(I2C_SCL_PIN, GPIO_MODE_INPUT);
        esp_rom_delay_us(20);
        sda_lvl = gpio_get_level(I2C_SDA_PIN);
        scl_lvl = gpio_get_level(I2C_SCL_PIN);
        ESP_LOGI(TAG, "After bus recovery: SDA=%d, SCL=%d", sda_lvl, scl_lvl);
    }

    if (sda_lvl == 0 || scl_lvl == 0) {
        ESP_LOGW(TAG, "WARNING: I2C line(s) still LOW! Verify AS5600 VCC (3.3V) & GND wiring.");
    }

    // 2. Initialize ESP-IDF I2C Master Bus
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

    ESP_LOGI(TAG, "I2C master bus initialized on SDA=GPIO %d, SCL=GPIO %d",
             I2C_SDA_PIN, I2C_SCL_PIN);

    // 3. Scan bus for connected devices
    i2c_bus_scan();

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
    return i2c_master_probe(bus, address, 50);
}

void i2c_bus_scan(void) {
    if (!s_bus_handle) return;
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    int found = 0;
    for (uint16_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus_handle, addr, 20) == ESP_OK) {
            const char *name = "Device";
            if (addr == 0x36) name = "AS5600 Magnetic Angle Sensor";
            else if (addr == 0x70) name = "TCA9548A I2C Multiplexer";
            else if (addr == 0x3C || addr == 0x3D) name = "SSD1306 OLED Display";
            ESP_LOGI(TAG, "  -> Found 0x%02X (%s)", addr, name);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  -> No I2C devices acknowledged probe. Check AS5600 power and wiring.");
    }
}
