#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MCP4725_ADDR_GND = 0x60,
        MCP4725_ADDR_VDD,
        MCP4725_ADDR_SDA,
        MCP4725_ADDR_SCL
    } mcp4725_address_t;

    typedef enum
    {
        MCP4725_EEPROM_BUSY = 0,
        MCP4725_EEPROM_READY
    } mcp4725_eeprom_status_t;

    typedef enum
    {
        MCP4725_PD_MODE_NORMAL = 0,
        MCP4725_PD_MODE_1K,
        MCP4725_PD_MODE_100K,
        MCP4725_PD_MODE_500K
    } mcp4725_pd_mode_t;

    esp_err_t mcp4725_init(i2c_master_dev_handle_t *dev_handle,
                           i2c_port_t port,
                           mcp4725_address_t address,
                           uint32_t speed_hz);
    esp_err_t mcp4725_read_eeprom_status(i2c_master_dev_handle_t *dev_handle,
                                         mcp4725_eeprom_status_t *eeprom_status);
    esp_err_t mcp4725_read_register_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                            mcp4725_pd_mode_t *pd_mode);
    esp_err_t mcp4725_read_eeprom_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                          mcp4725_pd_mode_t *pd_mode);
    esp_err_t mcp4725_read_register_data(i2c_master_dev_handle_t *dev_handle,
                                         uint16_t *data);
    esp_err_t mcp4725_read_eeprom_data(i2c_master_dev_handle_t *dev_handle,
                                       uint16_t *data);
    esp_err_t mcp4725_write_register_data(i2c_master_dev_handle_t *dev_handle,
                                          uint16_t data);
    esp_err_t mcp4725_write_eeprom_data(i2c_master_dev_handle_t *dev_handle,
                                        uint16_t data);
    esp_err_t mcp4725_write_register_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                             mcp4725_pd_mode_t pd_mode);
    esp_err_t mcp4725_write_eeprom_pd_mode(i2c_master_dev_handle_t *dev_handle,
                                           mcp4725_pd_mode_t pd_mode);

#ifdef __cplusplus
}
#endif
