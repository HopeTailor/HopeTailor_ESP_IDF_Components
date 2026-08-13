#ifndef HT_I2C_H
#define HT_I2C_H

#include <stdint.h>
#include <stddef.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

/**
 * @brief Default Hardware Pin Mapping:
 * - I2C_NUM_0 -> SDA: GPIO 21 | SCL: GPIO 22
 * - I2C_NUM_1 -> SDA: GPIO 18 | SCL: GPIO 19
 */
#define HT_I2C_PORT0_SDA_DEFAULT 21
#define HT_I2C_PORT0_SCL_DEFAULT 22

#define HT_I2C_PORT1_SDA_DEFAULT 18
#define HT_I2C_PORT1_SCL_DEFAULT 19

/**
 * @brief Configuration structure for the I2C Master Bus
 */
typedef struct {
    i2c_port_num_t i2c_port;    /* I2C port choice: I2C_NUM_0 or I2C_NUM_1 */
    uint32_t clk_speed_hz;      /* Bus speed in Hz (e.g., 400000 for 400kHz) */
} ht_i2c_config_t;

/**
 * @brief Initialize I2C master bus based on selected port.
 */
esp_err_t ht_i2c_init_bus(const ht_i2c_config_t *cfg, i2c_master_bus_handle_t *bus_handle);

/**
 * @brief Add an I2C slave device to an active bus.
 */
esp_err_t ht_i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed_hz, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Write a single byte to a register.
 */
esp_err_t ht_i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief Read multiple contiguous bytes from a starting register.
 */
esp_err_t ht_i2c_read_regs(i2c_master_dev_handle_t dev_handle, uint8_t start_reg, uint8_t *data_buf, size_t len);

#endif  /* HT_I2C_H */