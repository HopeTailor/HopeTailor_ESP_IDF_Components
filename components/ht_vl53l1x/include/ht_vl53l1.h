/**
 * @file ht_vl53l1x.h
 * @brief Lightweight, non-blocking VL53L1X ToF Sensor Driver for ESP-IDF.
 * @author HopeTailor
 * @note Relies on the custom ht_i2c wrapper library.
 */

#ifndef HT_VL53L1X_H
#define HT_VL53L1X_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ht_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Default 7-bit I2C address for the VL53L1X sensor.
 */
#define HT_VL53L1X_DEFAULT_ADDRESS 0x29

/**
 * @brief Distance modes of the sensor.
 * 
 * HOW TO CONFIGURE:
 * - MODE_SHORT: Max distance ~1.3m. Better ambient light immunity. Use for close-range obstacles.
 * - MODE_LONG: Max distance ~4.0m in the dark. Easily affected by strong ambient light (e.g., sunlight).
 */
typedef enum {
    HT_VL53L1X_MODE_SHORT = 1,
    HT_VL53L1X_MODE_LONG  = 2
} ht_vl53l1x_distance_mode_t;

/**
 * @brief Timing budget for a single measurement (in milliseconds).
 * 
 * HOW TO CONFIGURE:
 * - A larger timing budget increases accuracy and maximum range, but decreases the update rate.
 * - 15ms is ONLY allowed when using MODE_SHORT.
 * - 33ms is the minimum allowed timing budget for MODE_LONG.
 * - 50ms is highly recommended as a balanced default.
 */
typedef enum {
    HT_VL53L1X_TIMING_15MS  = 15,
    HT_VL53L1X_TIMING_20MS  = 20,
    HT_VL53L1X_TIMING_33MS  = 33,
    HT_VL53L1X_TIMING_50MS  = 50,
    HT_VL53L1X_TIMING_100MS = 100,
    HT_VL53L1X_TIMING_200MS = 200,
    HT_VL53L1X_TIMING_500MS = 500
} ht_vl53l1x_timing_budget_t;

/**
 * @brief Region of Interest (ROI) configuration.
 * 
 * HOW TO CONFIGURE:
 * - The sensor's receiver is a 16x16 SPAD array.
 * - width & height: Min 4, Max 16. (4x4 gives a narrow ~15 degree Field of View).
 * - center_spad: Defines where the ROI is centered. Default is 199. 
 *   (Can be changed to "look" slightly left/right/up/down without moving the sensor).
 */
typedef struct {
    uint8_t width;      /*!< Width of the ROI (4 to 16) */
    uint8_t height;     /*!< Height of the ROI (4 to 16) */
    uint8_t center_spad;/*!< Center SPAD number (0 to 255). Default: 199 */
} ht_vl53l1x_roi_config_t;

/**
 * @brief Main configuration structure to initialize the sensor.
 * 
 * HOW TO CONFIGURE:
 * - Populate this struct before calling ht_vl53l1x_init().
 * - Note: The Inter-measurement period is automatically handled behind the scenes 
 *   and synced with the timing budget for maximum performance.
 */
typedef struct {
    ht_vl53l1x_distance_mode_t distance_mode; /*!< Short or Long distance mode */
    ht_vl53l1x_timing_budget_t timing_budget; /*!< Time allowed to gather photons */
    ht_vl53l1x_roi_config_t roi;              /*!< FOV and target zone configuration */
} ht_vl53l1x_config_t;

/**
 * @brief Sensor Device Handle structure.
 * Holds the I2C device handle and keeps track of the current configuration.
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev; /*!< Handle assigned by ht_i2c_add_device */
    ht_vl53l1x_config_t current_cfg; /*!< Cached configuration to prevent redundant I2C writes */
} ht_vl53l1x_dev_t;

/**
 * @brief Sensor output data structure.
 */
typedef struct {
    uint16_t distance_mm;       /*!< Measured distance in millimeters */
    uint8_t  range_status;      /*!< 0 = Valid data, others = Error/Warning (e.g., Signal too weak) */
    uint16_t ambient_count_mcps;/*!< Ambient light noise detected (Mega counts per second) */
    uint16_t signal_count_mcps; /*!< Target return signal strength (Mega counts per second) */
} ht_vl53l1x_result_t;


/* =========================================================================
 *                              CORE FUNCTIONS
 * ========================================================================= */

/**
 * @brief Initializes the VL53L1X sensor.
 * This function adds the device to the I2C bus, boots the sensor firmware, 
 * and applies the provided configuration.
 * 
 * @param[in] bus_handle Active I2C master bus handle (from ht_i2c_init_bus).
 * @param[in] i2c_addr   Sensor I2C address (usually HT_VL53L1X_DEFAULT_ADDRESS).
 * @param[in] config     Pointer to the user configuration struct.
 * @param[out] dev       Pointer to an uninitialized device handle struct.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t ht_vl53l1x_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr, const ht_vl53l1x_config_t *config, ht_vl53l1x_dev_t *dev);

/**
 * @brief Starts continuous distance measurement.
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_start_ranging(ht_vl53l1x_dev_t *dev);

/**
 * @brief Stops continuous distance measurement to save power.
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_stop_ranging(ht_vl53l1x_dev_t *dev);

/**
 * @brief Checks if a new measurement is ready to be read.
 * Useful for non-blocking architectures (Polling mode via FreeRTOS tasks).
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @param[out] is_ready Sets to true if data is ready, false otherwise.
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_check_data_ready(ht_vl53l1x_dev_t *dev, bool *is_ready);

/**
 * @brief Reads the measurement result and clears the sensor's internal interrupt flag.
 * MUST be called after ht_vl53l1x_check_data_ready returns true.
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @param[out] result Pointer to store the measurement data.
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_get_result(ht_vl53l1x_dev_t *dev, ht_vl53l1x_result_t *result);

/**
 * @brief Dynamically updates the Region of Interest (ROI) at runtime.
 * Useful for scanning/sweeping an area without physically moving the sensor.
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @param[in] roi Pointer to the new ROI configuration.
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_set_roi(ht_vl53l1x_dev_t *dev, const ht_vl53l1x_roi_config_t *roi);

/* =========================================================================
 *                         CALIBRATION FUNCTIONS
 * ========================================================================= */

/**
 * @brief Applies an offset calibration value.
 * Used to compensate for the zero-distance error (e.g., if the sensor 
 * reads 5mm when touching a target, apply an offset of -5mm).
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @param[in] offset_mm Offset value in millimeters.
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_set_offset(ht_vl53l1x_dev_t *dev, int16_t offset_mm);

/**
 * @brief Applies a crosstalk calibration value.
 * Used to compensate for IR light bouncing back into the sensor from a 
 * cover glass or enclosure (which causes false close-range readings).
 * 
 * @param[in] dev Pointer to the initialized device handle.
 * @param[in] xtalk_cps Crosstalk value in counts per second (cps).
 * @return ESP_OK on success.
 */
esp_err_t ht_vl53l1x_set_crosstalk(ht_vl53l1x_dev_t *dev, uint16_t xtalk_cps);

#ifdef __cplusplus
}
#endif

#endif /* HT_VL53L1X_H */