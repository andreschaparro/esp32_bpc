#include <stdio.h>
#include "driver/i2c_master.h"
#include "mcp4725.h"

#define MCP4725_WR_DAC_REG 0x40
#define MCP4725_WR_DAC_REG_AND_EEPROM 0x60

esp_err_t mcp4725_init(i2c_master_dev_handle_t *dev_handle,
                       i2c_port_t port,
                       mcp4725_address_t address,
                       uint32_t speed_hz)
{
    if (dev_handle == NULL ||
        port >= I2C_NUM_MAX ||
        address < MCP4725_ADDR_GND || address > MCP4725_ADDR_SCL)
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
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
}

esp_err_t mcp4725_read_eeprom_status(i2c_master_dev_handle_t *dev_handle,
                                     mcp4725_eeprom_status_t *eeprom_status)
{
    if (dev_handle == NULL || eeprom_status == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd = 0;
    esp_err_t ret = i2c_master_receive(*dev_handle, &data_rd, 1, 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *eeprom_status = (mcp4725_eeprom_status_t)((data_rd & 0x80) >> 7);
    return ret;
}

esp_err_t mcp4725_read_register_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                        mcp4725_pd_mode_t *pd_mode)
{
    if (dev_handle == NULL || pd_mode == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd = 0;
    esp_err_t ret = i2c_master_receive(*dev_handle, &data_rd, 1, 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *pd_mode = (mcp4725_pd_mode_t)((data_rd & 0x06) >> 1);
    return ret;
}

esp_err_t mcp4725_read_eeprom_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                      mcp4725_pd_mode_t *pd_mode)
{
    if (dev_handle == NULL || pd_mode == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[4] = {0};
    esp_err_t ret = i2c_master_receive(*dev_handle, &data_rd, 4, 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *pd_mode = (mcp4725_pd_mode_t)((data_rd[3] & 0x60) >> 5);
    return ret;
}

esp_err_t mcp4725_read_register_data(i2c_master_dev_handle_t *dev_handle,
                                     uint16_t *data)
{
    if (dev_handle == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[3] = {0};
    esp_err_t ret = i2c_master_receive(*dev_handle, data_rd, 3, 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *data = ((uint16_t)data_rd[1] << 4) | ((data_rd[2] & 0xF0) >> 4);
    return ret;
}

esp_err_t mcp4725_read_eeprom_data(i2c_master_dev_handle_t *dev_handle,
                                   uint16_t *data)
{
    if (dev_handle == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_rd[5] = {0};
    esp_err_t ret = i2c_master_receive(*dev_handle, data_rd, 5, 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *data = ((uint16_t)(data_rd[3] & 0x0F) << 8) | data_rd[4];
    return ret;
}

esp_err_t mcp4725_write_register_data(i2c_master_dev_handle_t *dev_handle,
                                      uint16_t data)
{
    if (dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_wr[3] = {
        MCP4725_WR_DAC_REG,
        data >> 4,
        data << 4,
    };
    return i2c_master_transmit(*dev_handle, data_wr, 3, 1000);
}

esp_err_t mcp4725_write_eeprom_data(i2c_master_dev_handle_t *dev_handle,
                                    uint16_t data)
{
    if (dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_wr[3] = {
        MCP4725_WR_DAC_REG_AND_EEPROM,
        data >> 4,
        data << 4,
    };
    return i2c_master_transmit(*dev_handle, data_wr, 3, 1000);
}

esp_err_t mcp4725_write_register_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                         mcp4725_pd_mode_t pd_mode)
{
    if (dev_handle == NULL ||
        pd_mode < MCP4725_PD_MODE_NORMAL || pd_mode > MCP4725_PD_MODE_500K)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = mcp4725_read_register_data(dev_handle, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    uint8_t data_wr[3] = {
        MCP4725_WR_DAC_REG | ((uint8_t)pd_mode << 1),
        data >> 4,
        data << 4,
    };
    return i2c_master_transmit(*dev_handle, data_wr, 3, 1000);
}

esp_err_t mcp4725_write_eeprom_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                       mcp4725_pd_mode_t pd_mode)
{
    if (dev_handle == NULL ||
        pd_mode < MCP4725_PD_MODE_NORMAL || pd_mode > MCP4725_PD_MODE_500K)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = mcp4725_read_eeprom_data(dev_handle, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    uint8_t data_wr[3] = {
        MCP4725_WR_DAC_REG_AND_EEPROM | ((uint8_t)pd_mode << 1),
        data >> 4,
        data << 4,
    };
    return i2c_master_transmit(*dev_handle, data_wr, 3, 1000);
}
