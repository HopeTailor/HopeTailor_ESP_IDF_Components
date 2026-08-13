#include "ht_i2c.h"
#include "esp_log.h"

static const char *TAG = "HT_I2C";

esp_err_t ht_i2c_init_bus(const ht_i2c_config_t *cfg, i2c_master_bus_handle_t *bus_handle) {
    if(cfg == NULL || bus_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int sda_pin;
    int scl_pin;

    // Switch-case to assign standard default pins based on selected I2C Port
    switch(cfg->i2c_port) {
        case I2C_NUM_0:
            sda_pin = HT_I2C_PORT0_SDA_DEFAULT;
            scl_pin = HT_I2C_PORT0_SCL_DEFAULT;
            break;
        case I2C_NUM_1:
            sda_pin = HT_I2C_PORT1_SDA_DEFAULT;
            scl_pin = HT_I2C_PORT1_SCL_DEFAULT;
            break;
        default:    
            ESP_LOGE(TAG, "Invalid I2C port selected!");
            return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = cfg->i2c_port,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_LOGI(TAG, "Initializing I2C Port %d -> SDA: GPIO %d, SCL: GPIO %d", cfg->i2c_port, sda_pin, scl_pin);

    return i2c_new_master_bus(&bus_cfg, bus_handle);
}

esp_err_t ht_i2c_add_device(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed_hz, i2c_master_dev_handle_t *dev_handle) {
    if(bus_handle == NULL || dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = (clk_speed_hz > 0) ? clk_speed_hz : 400000,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
}

esp_err_t ht_i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
}

esp_err_t ht_i2c_read_regs(i2c_master_dev_handle_t dev_handle, uint8_t start_reg, uint8_t *data_buf, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &start_reg, 1, data_buf, len, -1);
}