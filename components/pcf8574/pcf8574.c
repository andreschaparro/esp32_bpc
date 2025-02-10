#include <stdio.h>
#include "driver/i2c_master.h"
#include "pcf8574.h"

static esp_err_t s_pcf8574_read_port(i2c_master_dev_handle_t *dev_handle,
                                     uint8_t *data);
static esp_err_t s_pcf8574_write_port(i2c_master_dev_handle_t *dev_handle,
                                      uint8_t data);

esp_err_t pcf8574_init(pcf8574_device_t *device,
                       i2c_port_t port,
                       pcf8574_address_t address,
                       uint32_t speed_hz)
{
    if (device == NULL ||
        port >= I2C_NUM_MAX ||
        address < PCF8574_ADDR_LLL || address > PCF8574_ADDR_HHH)
    {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_master_get_bus_handle(port, &bus_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = (uint16_t)address,
        .scl_speed_hz = speed_hz,
        .scl_wait_us = 1000,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &device->dev_handle);
    if (ret != ESP_OK)
    {
        return ret;
    }
    device->port_status = 0xFF;
    return s_pcf8574_write_port(&device->dev_handle, device->port_status);
}

esp_err_t pcf8574_read_pin(pcf8574_device_t *device,
                           pcf8574_pin_t pin,
                           pcf8574_level_t *level)
{
    if (device == NULL ||
        pin < PCF8574_PIN_P0 || pin > PCF8574_PIN_P7 ||
        level == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data = 0;
    esp_err_t ret = s_pcf8574_read_port(&device->dev_handle, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *level = (data & (uint8_t)(0x01 << pin))
                 ? PCF8574_HIGH
                 : PCF8574_LOW;
    return ret;
}

esp_err_t pcf8574_write_pin(pcf8574_device_t *device,
                            pcf8574_pin_t pin,
                            pcf8574_level_t level)
{
    if (device == NULL ||
        pin < PCF8574_PIN_P0 || pin > PCF8574_PIN_P7 ||
        (level != PCF8574_LOW && level != PCF8574_HIGH))
    {
        return ESP_ERR_INVALID_ARG;
    }
    device->port_status = (level == PCF8574_LOW)
                              ? (device->port_status & ~(uint8_t)(0x01 << pin))
                              : (device->port_status | (uint8_t)(0x01 << pin));
    return s_pcf8574_write_port(&device->dev_handle, device->port_status);
}

esp_err_t pcf8574_toggle_pin(pcf8574_device_t *device,
                             pcf8574_pin_t pin)
{
    if (device == NULL ||
        pin < PCF8574_PIN_P0 || pin > PCF8574_PIN_P7)
    {
        return ESP_ERR_INVALID_ARG;
    }
    device->port_status ^= (uint8_t)(0x01 << pin);
    return s_pcf8574_write_port(&device->dev_handle, device->port_status);
}

static esp_err_t s_pcf8574_read_port(i2c_master_dev_handle_t *dev_handle,
                                     uint8_t *data)
{
    if (dev_handle == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_receive(*dev_handle, data, 1, 1000);
}

static esp_err_t s_pcf8574_write_port(i2c_master_dev_handle_t *dev_handle,
                                      uint8_t data)
{
    if (dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_transmit(*dev_handle, &data, 1, 1000);
}
