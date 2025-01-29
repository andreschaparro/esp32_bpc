#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mcp4725.h"

// ================================ Private Defines ===============================================

// I2C master configuration
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define I2C_TIMEOUT_MS 1000       // I2C timeout in milliseconds

// ================================ Public Functions Definitions ==================================

// Initialize the device
esp_err_t mcp4725_init(mcp4725_dev_t *dev, i2c_port_t port, uint16_t addr, gpio_num_t sda, gpio_num_t scl)
{
    if (dev == NULL ||
        port >= I2C_NUM_MAX ||
        addr < MCP4725_I2C_ADDR_GND || addr > MCP4725_I2C_ADDR_SCL ||
        sda <= GPIO_NUM_NC || sda >= GPIO_NUM_MAX ||
        scl <= GPIO_NUM_NC || scl >= GPIO_NUM_MAX)
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
    *busy = (((data_rd >> 7) & 0x01) == 0);
    return ret;
}

// Get the power-down mode from register or EEPROM
esp_err_t mcp4725_get_pd_mode(mcp4725_dev_t *dev, bool eeprom, mcp4725_pd_mode_t *pd)
{
    if (dev == NULL || pd == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[4] = {0};
    esp_err_t ret = eeprom
                        ? i2c_master_receive(dev->i2c_dev, data_rd, 4, I2C_TIMEOUT_MS)
                        : i2c_master_receive(dev->i2c_dev, data_rd, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *pd = (mcp4725_pd_mode_t)((eeprom ? data_rd[3] >> 5 : data_rd[0] >> 1) & 0x03);
    return ret;
}

esp_err_t mcp4725_set_pd_mode(mcp4725_dev_t *dev, bool eeprom, mcp4725_pd_mode_t pd)
{
    return ESP_OK;
}

// Get the raw value from register or EEPROM
esp_err_t mcp4725_get_raw(mcp4725_dev_t *dev, bool eeprom, uint16_t *val)
{
    if (dev == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[5] = {0};
    esp_err_t ret = eeprom
                        ? i2c_master_receive(dev->i2c_dev, data_rd, 5, I2C_TIMEOUT_MS)
                        : i2c_master_receive(dev->i2c_dev, data_rd, 3, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *val = (uint16_t)((eeprom
                           ? ((data_rd[3] & 0x0F) << 8) | data_rd[4]
                           : (data_rd[1] << 4) | ((data_rd[2] & 0xF0) >> 4)));
    return ret;
}

esp_err_t mcp4725_set_raw(mcp4725_dev_t *dev, bool eeprom, uint16_t val)
{
    return ESP_OK;
}

// Get the voltage from register or EEPROM
esp_err_t mcp4725_get_volt(mcp4725_dev_t *dev, bool eeprom, float vdd, float *volt)
{
    uint16_t aux = 0;
    mcp4725_get_raw(dev, eeprom, &aux);
    *volt = (float)aux * (vdd / 4095.0f);
    return ESP_OK;
}

esp_err_t mcp4725_set_volt(mcp4725_dev_t *dev, bool eeprom, float volt)
{
    return ESP_OK;
}