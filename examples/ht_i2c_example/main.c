#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ht_i2c.h"

static const char *TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Starting I2C Master Bus initialization example...");

    // Initialize I2C Port 0 (Auto-assigns to SDA: GPIO 21, SCL: GPIO 22)
    ht_i2c_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .clk_speed_hz = 400000,
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = ht_i2c_init_bus(&i2c_cfg, &bus_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C Bus initialized successfully!");
    } else {
        ESP_LOGE(TAG, "Failed to initialize I2C Bus: %s", esp_err_to_name(err));
    }
}