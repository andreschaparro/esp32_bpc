#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ================================ Public Defines ============================================

#define MCP4725_I2C_ADDR_GND 0x60 // I2C address with ADDR pin connected to GND (default)
#define MCP4725_I2C_ADDR_VDD 0x61 // I2C address with ADDR pin connected to VDD
#define MCP4725_I2C_ADDR_SDA 0x62 // I2C address with ADDR pin connected to SDA
#define MCP4725_I2C_ADDR_SCL 0x63 // I2C address with ADDR pin connected to SCL

    // ================================ Public Data Types =========================================

    typedef struct
    {
        i2c_master_dev_handle_t i2c_dev;
        i2c_port_t i2c_port;
        uint16_t i2c_addr;
    } mcp4725_dev_t;

    typedef enum
    {
        MCP4725_PD_MODE_NORMAL = 0, // Normal mode
        MCP4725_PD_MODE_1K,         // Power-down mode with 1K ohm pull-up resistor
        MCP4725_PD_MODE_100K,       // Power-down mode with 100K ohm pull-up resistor
        MCP4725_PD_MODE_500K,       // Power-down mode with 500K ohm pull-up resistor
    } mcp4725_pd_mode_t;

    // ================================ Public Functions Declaration ==============================

    esp_err_t mcp4725_init(mcp4725_dev_t *dev, i2c_port_t port, uint16_t addr, gpio_num_t sda, gpio_num_t scl);
    esp_err_t mcp4725_eeprom_busy(mcp4725_dev_t *dev, bool *busy);
    esp_err_t mcp4725_get_pd_mode(mcp4725_dev_t *dev, bool eeprom, mcp4725_pd_mode_t *pd);
    esp_err_t mcp4725_set_pd_mode(mcp4725_dev_t *dev, bool eeprom, mcp4725_pd_mode_t pd);
    esp_err_t mcp4725_get_raw(mcp4725_dev_t *dev, bool eeprom, uint16_t *val);
    esp_err_t mcp4725_set_raw(mcp4725_dev_t *dev, bool eeprom, uint16_t val);
    esp_err_t mcp4725_get_volt(mcp4725_dev_t *dev, bool eeprom, float vdd, float *volt);
    esp_err_t mcp4725_set_volt(mcp4725_dev_t *dev, bool eeprom, float volt);

#ifdef __cplusplus
}
#endif