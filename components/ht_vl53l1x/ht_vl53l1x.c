/**
 * @file ht_vl53l1x.c
 * @brief Lightweight, non-blocking VL53L1X ToF Sensor Driver for ESP-IDF.
 * @author HopeTailor
 */

#include "ht_vl53l1x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HT_VL53L1X";

/* =========================================================================
 *                      VL53L1X Register Addresses
 * ========================================================================= */
#define VL53L1X_REG_SOFT_RESET                  0x010F
#define VL53L1X_REG_FIRMWARE_SYSTEM_STATUS      0x00E5
#define VL53L1X_REG_DEFAULT_CONFIG_START        0x002D

/* 
 * @brief Default STMicroelectronics configuration array (109 bytes).
 * This "magic array" MUST be written to the sensor at boot time starting 
 * from register 0x002D to properly configure the internal firmware.
 */
static const uint8_t VL53L1X_DEFAULT_CONFIGURATION[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 
    0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x02, 0x00, 
    0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* =========================================================================
 *                              CORE FUNCTIONS
 * ========================================================================= */

esp_err_t ht_vl53l1x_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr, const ht_vl53l1x_config_t *config, ht_vl53l1x_dev_t *dev) {
    if (bus_handle == NULL || config == NULL || dev == NULL) {
        ESP_LOGE(TAG, "Invalid arguments passed to init");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;

    // 1. Add device to the I2C bus with 400kHz Fast-mode capability
    err = ht_i2c_add_device(bus_handle, i2c_addr, 400000, &dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return err;
    }

    // 2. Perform a Software Reset (Required for a clean boot)
    ESP_LOGI(TAG, "Resetting sensor...");
    err |= ht_i2c_write_reg16(dev->i2c_dev, VL53L1X_REG_SOFT_RESET, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1)); // Wait 1ms
    err |= ht_i2c_write_reg16(dev->i2c_dev, VL53L1X_REG_SOFT_RESET, 0x01);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 3. Wait for the firmware to boot up completely
    uint8_t fw_status = 0;
    int timeout = 100; // 100 * 10ms = 1 second max timeout
    
    while ((fw_status & 0x01) == 0 && timeout > 0) {
        ht_i2c_read_regs16(dev->i2c_dev, VL53L1X_REG_FIRMWARE_SYSTEM_STATUS, &fw_status, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    if (timeout == 0) {
        ESP_LOGE(TAG, "Sensor firmware boot timeout! Check wiring and power.");
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Firmware booted successfully");

    // 4. Load the magic default configuration array via Burst Write
    err = ht_i2c_write_regs16(dev->i2c_dev, VL53L1X_REG_DEFAULT_CONFIG_START, VL53L1X_DEFAULT_CONFIGURATION, sizeof(VL53L1X_DEFAULT_CONFIGURATION));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load default configuration");
        return err;
    }

    // 5. Cache the user config (We will apply them in the next step)
    dev->current_cfg = *config;
    
    // TODO: Step 2 -> Apply distance mode, timing budget, and ROI based on 'dev->current_cfg'

    ESP_LOGI(TAG, "Basic initialization completed");
    return ESP_OK;
}