#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mcp4725.h"

// ================================ Private Defines ===============================================

// I2C master configuration
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define I2C_TIMEOUT_MS 1000       // I2C timeout in milliseconds

// MCP4725
#define MCP4725_CMD_WR_DAC 0x40    // Write to DAC register
#define MCP4725_CMD_WR_EEPROM 0x60 // Write to EEPROM register
#define MCP4725_EEPROM_READY 0x80  // EEPROM ready bit

// ================================ Public Functions Definitions ==================================

// Initialize the device
esp_err_t mcp4725_init(mcp4725_dev_t *dev, i2c_port_t port, uint16_t addr)
{
    if (dev == NULL ||
        port >= I2C_NUM_MAX ||
        addr < MCP4725_I2C_ADDR_GND || addr > MCP4725_I2C_ADDR_SCL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    dev->i2c_port = port;
    dev->i2c_addr = addr;
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_master_get_bus_handle(dev->i2c_port, &bus_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev->i2c_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 1000,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
}

// Check if the EEPROM is busy
esp_err_t mcp4725_eeprom_busy(mcp4725_dev_t *dev, bool *busy)
{
    if (dev == NULL || busy == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd = 0;
    esp_err_t ret = i2c_master_receive(dev->i2c_dev, &data_rd, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *busy = (data_rd & MCP4725_EEPROM_READY) == 0;
    return ret;
}

// Get the power-down mode from register or EEPROM
esp_err_t mcp4725_get_pd_mode(mcp4725_dev_t *dev, mcp4725_pd_mode_t *pd_mode, bool eeprom)
{
    if (dev == NULL || pd_mode == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[4] = {0};
    esp_err_t ret = i2c_master_receive(dev->i2c_dev, data_rd, eeprom ? 4 : 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *pd_mode = (mcp4725_pd_mode_t)((eeprom ? data_rd[3] >> 5 : data_rd[0] >> 1) & 0x03);
    return ret;
}

// Set the power-down mode from register or EEPROM
esp_err_t mcp4725_set_pd_mode(mcp4725_dev_t *dev, mcp4725_pd_mode_t pd_mode, bool eeprom)
{
    if (dev == NULL ||
        pd_mode < MCP4725_PD_MODE_NORMAL || pd_mode > MCP4725_PD_MODE_500K)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t val = 0;
    esp_err_t ret = mcp4725_get_raw(dev, &val, eeprom);
    if (ret != ESP_OK)
    {
        return ret;
    }
    uint8_t data_wr[3] = {
        (eeprom ? MCP4725_CMD_WR_EEPROM : MCP4725_CMD_WR_DAC) | ((uint8_t)pd_mode << 1),
        val >> 4,
        val << 4,
    };
    return i2c_master_transmit(dev->i2c_dev, data_wr, 3, I2C_TIMEOUT_MS);
}

// Get the raw value from register or EEPROM
esp_err_t mcp4725_get_raw(mcp4725_dev_t *dev, uint16_t *val, bool eeprom)
{
    if (dev == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[5] = {0};
    esp_err_t ret = i2c_master_receive(dev->i2c_dev, data_rd, eeprom ? 5 : 3, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *val = eeprom
               ? ((uint16_t)(data_rd[3] & 0x0F) << 8) | data_rd[4]
               : ((uint16_t)data_rd[1] << 4) | ((data_rd[2] & 0xF0) >> 4);
    return ret;
}

// Set the raw value to register or EEPROM
esp_err_t mcp4725_set_raw(mcp4725_dev_t *dev, uint16_t val, bool eeprom)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_wr[3] = {
        eeprom ? MCP4725_CMD_WR_EEPROM : MCP4725_CMD_WR_DAC,
        val >> 4,
        val << 4,
    };
    return i2c_master_transmit(dev->i2c_dev, data_wr, 3, I2C_TIMEOUT_MS);
}

// Get the voltage from register or EEPROM
esp_err_t mcp4725_get_volt(mcp4725_dev_t *dev, float *volt, float vdd, bool eeprom)
{
    if (dev == NULL || volt == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t val = 0;
    esp_err_t ret = mcp4725_get_raw(dev, &val, eeprom);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *volt = (vdd / (float)MCP4725_MAX_VALUE) * (float)val;
    return ret;
}

// Set the voltage to register or EEPROM
esp_err_t mcp4725_set_volt(mcp4725_dev_t *dev, float volt, float vdd, bool eeprom)
{
    return mcp4725_set_raw(dev, (uint16_t)((volt / vdd) * MCP4725_MAX_VALUE), eeprom);
}