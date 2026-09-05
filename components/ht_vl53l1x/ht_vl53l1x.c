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

/* =========================================================================
 *                    CORE CONTROL FUNCTIONS (Step 2)
 * ========================================================================= */

esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG, "Starting continuous ranging...");
    /* 
     * Write 0x40 to SYSTEM_MODE_START (0x0087) 
     * This commands the sensor's internal state machine to start firing the laser.
     */
    return ht_i2c_write_reg16(dev->i2c_dev, VL53L1X_REG_SYSTEM_MODE_START, 0x40);
}

esp_err_t ht_vl53l1x_stop_ranging(ht_vl53l1x_dev_t *dev) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG, "Stopping ranging to save power...");
    /* Write 0x00 to SYSTEM_MODE_START to halt measurements */
    return ht_i2c_write_reg16(dev->i2c_dev, VL53L1X_REG_SYSTEM_MODE_START, 0x00);
}

esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, bool *is_ready) {
    if (dev == NULL || is_ready == NULL) return ESP_ERR_INVALID_ARG;
    
    uint8_t status = 0;
    /* Read the hardware interrupt status register (0x0031) */
    esp_err_t err = ht_i2c_read_regs16(dev->i2c_dev, VL53L1X_REG_GPIO_TIO_HV_STATUS, &status, 1);
    if (err != ESP_OK) return err;

    /* 
     * Based on our default 109-byte configuration, bit 0 of this register 
     * will flip to 1 when a new valid measurement is available.
     */
    *is_ready = (status & 0x01) == 1;
    return ESP_OK;
}

esp_err_t ht_vl53l1x_get_result(ht_vl53l1x_dev_t *dev, ht_vl53l1x_result_t *result) {
    if (dev == NULL || result == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    uint8_t data_buf[2];

    /* 1. Read the 16-bit distance in millimeters from register 0x0096 */
    err = ht_i2c_read_regs16(dev->i2c_dev, VL53L1X_REG_RESULT_DISTANCE_MM, data_buf, 2);
    if (err != ESP_OK) return err;
    
    /* Combine the two bytes (High Byte << 8 | Low Byte) */
    result->distance_mm = (data_buf[0] << 8) | data_buf[1];

    /* 2. Read the Range Status from register 0x0089 to check for errors/noise */
    err = ht_i2c_read_regs16(dev->i2c_dev, VL53L1X_REG_RESULT_RANGE_STATUS, &result->range_status, 1);
    if (err != ESP_OK) return err;
    
    /* Standard ST practice: Mask the status byte to get the exact error code */
    result->range_status = result->range_status & 0x1F;

    /* 
     * 3. CRITICAL STEP: Clear the internal interrupt flag.
     * If we don't write 0x01 to register 0x0086, the sensor will freeze 
     * and never take another measurement!
     */
    err = ht_i2c_write_reg16(dev->i2c_dev, VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    
    return err;
}

esp_err_t ht_vl53l1x_set_roi(ht_vl53l1x_dev_t *dev, const ht_vl53l1x_roi_config_t *roi) {
    if (dev == NULL || roi == NULL) return ESP_ERR_INVALID_ARG;

    /* Boundary check: Width and Height must be strictly between 4 and 16 */
    uint8_t w = (roi->width > 16) ? 16 : ((roi->width < 4) ? 4 : roi->width);
    uint8_t h = (roi->height > 16) ? 16 : ((roi->height < 4) ? 4 : roi->height);
    
    /* 
     * The sensor expects a single byte for size.
     * Format: High nibble = Height - 1, Low nibble = Width - 1
     */
    uint8_t roi_xy_size = ((h - 1) << 4) | (w - 1);

    esp_err_t err = ESP_OK;
    
    /* 0x007F: Set ROI Center SPAD */
    err |= ht_i2c_write_reg16(dev->i2c_dev, 0x007F, roi->center_spad);
    
    /* 0x007E: Set ROI Size (XY) */
    err |= ht_i2c_write_reg16(dev->i2c_dev, 0x007E, roi_xy_size);
    
    if (err == ESP_OK) {
        dev->current_cfg.roi = *roi; /* Update the cached struct */
        ESP_LOGI(TAG, "ROI dynamically updated -> Center: %d, Size: %dx%d", roi->center_spad, w, h);
    }
    
    return err;
}

