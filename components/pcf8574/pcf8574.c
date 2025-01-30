#include <stdio.h>
#include "driver/i2c_master.h"
#include "pcf8574.h"

// ================================ Private Defines ===============================================

// I2C master configuration
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define I2C_TIMEOUT_MS 1000       // I2C timeout in milliseconds

// ================================ Private Functions Declarations ================================

static esp_err_t pcf8574_get_port(pcf8574_dev_t *dev, uint8_t *val);
static esp_err_t pcf8574_set_port(pcf8574_dev_t *dev, uint8_t val);

// ================================ Public Functions Definitions ==================================

// Initialize the device
esp_err_t pcf8574_init(pcf8574_dev_t *dev, i2c_port_t port, uint16_t addr)
{
    if (dev == NULL ||
        port >= I2C_NUM_MAX ||
        (addr != PCF8574_I2C_ADDR_LLL &&
         addr != PCF8574_I2C_ADDR_LLH &&
         addr != PCF8574_I2C_ADDR_LHL &&
         addr != PCF8574_I2C_ADDR_LHH &&
         addr != PCF8574_I2C_ADDR_HLL &&
         addr != PCF8574_I2C_ADDR_HLH &&
         addr != PCF8574_I2C_ADDR_HHL &&
         addr != PCF8574_I2C_ADDR_HHH))
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

// Get the pin value
esp_err_t pcf8574_get_pin(pcf8574_dev_t *dev, pcf8574_pin_t pin, pcf8574_state_t *val)
{
    if (dev == NULL || val == NULL || pin < PCF8574_P0 || pin > PCF8574_P7)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd;
    esp_err_t ret = pcf8574_get_port(dev, &data_rd);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *val = (pcf8574_state_t)((data_rd >> pin) & 0x01);
    return ret;
}

// Set the pin value
esp_err_t pcf8574_set_pin(pcf8574_dev_t *dev, pcf8574_pin_t pin, pcf8574_state_t val)
{
    if (dev == NULL ||
        pin < PCF8574_P0 || pin > PCF8574_P7 ||
        (val != PCF8574_LOW && val != PCF8574_HIGH))
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd;
    esp_err_t ret = pcf8574_get_port(dev, &data_rd);
    if (ret != ESP_OK)
    {
        return ret;
    }
    uint8_t data_wr = val ? (data_rd | (uint8_t)(0x01 << pin)) : (data_rd & (uint8_t)~(0x01 << pin));
    return pcf8574_set_port(dev, data_wr);
}

// Set the pin direction
esp_err_t pcf8574_set_pin_dir(pcf8574_dev_t *dev, pcf8574_pin_t pin, pcf8574_dir_t dir)
{
    return ESP_OK;
}

// Toggle the pin value
esp_err_t pcf8574_toogle_pin(pcf8574_dev_t *dev, pcf8574_pin_t pin)
{
    if (dev == NULL || pin < PCF8574_P0 || pin > PCF8574_P7)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd;
    esp_err_t ret = pcf8574_get_port(dev, &data_rd);
    if (ret != ESP_OK)
    {
        return ret;
    }
    uint8_t data_wr = data_rd ^ (uint8_t)(0x01 << pin);
    return pcf8574_set_port(dev, data_wr);
}

// ================================ Private Functions Definitions =================================

// Get the port value
static esp_err_t pcf8574_get_port(pcf8574_dev_t *dev, uint8_t *val)
{
    if (dev == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_receive(dev->i2c_dev, val, 1, I2C_TIMEOUT_MS);
}

// Set the port value
static esp_err_t pcf8574_set_port(pcf8574_dev_t *dev, uint8_t val)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_wr = val;
    return i2c_master_transmit(dev->i2c_dev, &data_wr, 1, I2C_TIMEOUT_MS);
}