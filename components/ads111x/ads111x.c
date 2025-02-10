#include <stdio.h>
#include "driver/i2c_master.h"
#include "ads111x.h"

static esp_err_t s_ads111x_read_register(i2c_master_dev_handle_t *dev_handle,
                                         ads111x_reg_t reg,
                                         uint16_t *data);
static esp_err_t s_ads111x_write_register(i2c_master_dev_handle_t *dev_handle,
                                          ads111x_reg_t reg,
                                          uint16_t data);

esp_err_t ads111x_init(i2c_master_dev_handle_t *dev_handle,
                       i2c_port_num_t port,
                       ads111x_address_t address,
                       uint32_t speed_hz)
{
    if (dev_handle == NULL ||
        port >= I2C_NUM_MAX ||
        address < ADS111X_ADDR_GND || address > ADS111X_ADDR_SCL)
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

esp_err_t ads111x_read_os(i2c_master_dev_handle_t *dev_handle,
                          ads111x_os_t *os)
{
    if (dev_handle == NULL || os == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *os = (ads111x_os_t)((data >> 15) & 0x01);
    return ret;
}

esp_err_t ads111x_start_single_conv(i2c_master_dev_handle_t *dev_handle)
{
    if (dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data |= 0x8000;
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_mux(i2c_master_dev_handle_t *dev_handle,
                           ads111x_mux_t *mux)
{
    if (dev_handle == NULL || mux == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *mux = (ads111x_mux_t)((data >> 12) & 0x07);
    return ret;
}

esp_err_t ads111x_write_mux(i2c_master_dev_handle_t *dev_handle,
                            ads111x_mux_t mux)
{
    if (dev_handle == NULL ||
        mux < ADS111X_MUX_AIN0_AIN1 || mux > ADS111X_MUX_AIN3_GND)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x07 << 12)) | ((mux & 0x07) << 12);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_pga(i2c_master_dev_handle_t *dev_handle,
                           ads111x_pga_t *pga)
{
    if (dev_handle == NULL || pga == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *pga = (ads111x_pga_t)((data >> 9) & 0x07);
    return ret;
}

esp_err_t ads111x_write_pga(i2c_master_dev_handle_t *dev_handle,
                            ads111x_pga_t pga)
{
    if (dev_handle == NULL ||
        pga < ADS111X_PGA_6_144V || pga > ADS111X_PGA_0_256V)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x07 << 9)) | ((pga & 0x07) << 9);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_mode(i2c_master_dev_handle_t *dev_handle,
                            ads111x_mode_t *mode)
{
    if (dev_handle == NULL || mode == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *mode = (ads111x_mode_t)((data >> 8) & 0x01);
    return ret;
}

esp_err_t ads111x_write_mode(i2c_master_dev_handle_t *dev_handle,
                             ads111x_mode_t mode)
{
    if (dev_handle == NULL ||
        (mode != ADS111X_MODE_CONTINUOUS && mode != ADS111X_MODE_SINGLE_SHOT))
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x01 << 8)) | ((mode & 0x01) << 8);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_dr(i2c_master_dev_handle_t *dev_handle,
                          ads111x_dr_t *dr)
{
    if (dev_handle == NULL || dr == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *dr = (ads111x_dr_t)((data >> 5) & 0x07);
    return ret;
}

esp_err_t ads111x_write_dr(i2c_master_dev_handle_t *dev_handle,
                           ads111x_dr_t dr)
{
    if (dev_handle == NULL ||
        dr < ADS111X_DR_8SPS || dr > ADS111X_DR_860SPS)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x07 << 5)) | ((dr & 0x07) << 5);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_comp_mode(i2c_master_dev_handle_t *dev_handle,
                                 ads111x_comp_mode_t *comp_mode)
{
    if (dev_handle == NULL || comp_mode == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *comp_mode = (ads111x_comp_mode_t)((data >> 4) & 0x01);
    return ret;
}

esp_err_t ads111x_write_comp_mode(i2c_master_dev_handle_t *dev_handle,
                                  ads111x_comp_mode_t comp_mode)
{
    if (dev_handle == NULL ||
        (comp_mode != ADS111X_COMP_MODE_TRADITIONAL && comp_mode != ADS111X_COMP_MODE_WINDOW))
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x01 << 4)) | ((comp_mode & 0x01) << 4);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_comp_pol(i2c_master_dev_handle_t *dev_handle,
                                ads111x_comp_pol_t *comp_pol)
{
    if (dev_handle == NULL || comp_pol == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *comp_pol = (ads111x_comp_pol_t)((data >> 3) & 0x01);
    return ret;
}

esp_err_t ads111x_write_comp_pol(i2c_master_dev_handle_t *dev_handle,
                                 ads111x_comp_pol_t comp_pol)
{
    if (dev_handle == NULL ||
        (comp_pol != ADS111X_COMP_POL_LOW && comp_pol != ADS111X_COMP_POL_HIGH))
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x01 << 3)) | ((comp_pol & 0x01) << 3);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_comp_lat(i2c_master_dev_handle_t *dev_handle,
                                ads111x_comp_lat_t *comp_lat)
{
    if (dev_handle == NULL || comp_lat == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *comp_lat = (ads111x_comp_lat_t)((data >> 2) & 0x01);
    return ret;
}

esp_err_t ads111x_write_comp_lat(i2c_master_dev_handle_t *dev_handle,
                                 ads111x_comp_lat_t comp_lat)
{
    if (dev_handle == NULL ||
        (comp_lat != ADS111X_COMP_NON_LATCH && comp_lat != ADS111X_COMP_LATCH))
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~(0x01 << 2)) | ((comp_lat & 0x01) << 2);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_comp_que(i2c_master_dev_handle_t *dev_handle,
                                ads111x_comp_que_t *comp_que)
{
    if (dev_handle == NULL || comp_que == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *comp_que = (ads111x_comp_que_t)(data & 0x03);
    return ret;
}

esp_err_t ads111x_write_comp_que(i2c_master_dev_handle_t *dev_handle,
                                 ads111x_comp_que_t comp_que)
{
    if (dev_handle == NULL ||
        comp_que < ADS111X_COMP_QUE_1 || comp_que > ADS111X_COMP_QUE_DISABLE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t data = 0;
    esp_err_t ret = s_ads111x_read_register(dev_handle, ADS111X_CONFIG_REG, &data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (data & ~0x03) | (comp_que & 0x03);
    return s_ads111x_write_register(dev_handle, ADS111X_CONFIG_REG, data);
}

esp_err_t ads111x_read_comp_lo_thresh(i2c_master_dev_handle_t *dev_handle,
                                      int16_t *val)
{
    if (dev_handle == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return s_ads111x_read_register(dev_handle, ADS111X_LO_THRESH_REG, (uint16_t *)val);
}

esp_err_t ads111x_write_comp_lo_thresh(i2c_master_dev_handle_t *dev_handle,
                                       int16_t val)
{
    if (dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return s_ads111x_write_register(dev_handle, ADS111X_LO_THRESH_REG, (uint16_t)val);
}

esp_err_t ads111x_read_comp_hi_thresh(i2c_master_dev_handle_t *dev_handle,
                                      int16_t *val)
{
    if (dev_handle == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return s_ads111x_read_register(dev_handle, ADS111X_HI_THRESH_REG, (uint16_t *)val);
}

esp_err_t ads111x_write_comp_hi_thresh(i2c_master_dev_handle_t *dev_handle,
                                       int16_t val)
{
    if (dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return s_ads111x_write_register(dev_handle, ADS111X_HI_THRESH_REG, (uint16_t)val);
}

esp_err_t ads111x_read_conv(i2c_master_dev_handle_t *dev_handle,
                            int16_t *val)
{
    if (dev_handle == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return s_ads111x_read_register(dev_handle, ADS111X_CONV_REG, (uint16_t *)val);
}

static esp_err_t s_ads111x_read_register(i2c_master_dev_handle_t *dev_handle,
                                         ads111x_reg_t reg,
                                         uint16_t *data)
{
    if (dev_handle == NULL ||
        reg < ADS111X_CONV_REG || reg > ADS111X_HI_THRESH_REG ||
        data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_wr = reg;
    uint8_t data_rd[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(*dev_handle, &data_wr, 1, data_rd, 2, 1000);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *data = ((uint16_t)data_rd[0] << 8) | data_rd[1];
    return ret;
}

esp_err_t ads111x_read_lsb_size(i2c_master_dev_handle_t *dev_handle,
                                float *val)
{
    if (dev_handle == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    ads111x_pga_t pga;
    esp_err_t ret = ads111x_read_pga(dev_handle, &pga);
    if (ret != ESP_OK)
    {
        return ret;
    }
    else if (pga < ADS111X_PGA_6_144V || pga > ADS111X_PGA_0_256V)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }
    static const float s_ads111x_pga_lsb_size[] = {
        [ADS111X_PGA_6_144V] = 187.5f,
        [ADS111X_PGA_4_096V] = 125.0f,
        [ADS111X_PGA_2_048V] = 62.5f,
        [ADS111X_PGA_1_024V] = 31.25f,
        [ADS111X_PGA_0_512V] = 15.625f,
        [ADS111X_PGA_0_256V] = 7.8125f,
    };
    *val = s_ads111x_pga_lsb_size[pga];
    return ret;
}

static esp_err_t s_ads111x_write_register(i2c_master_dev_handle_t *dev_handle,
                                          ads111x_reg_t reg,
                                          uint16_t data)
{
    if (dev_handle == NULL ||
        reg < ADS111X_CONV_REG || reg > ADS111X_HI_THRESH_REG)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data_wr[3] = {reg,
                          (uint8_t)(data >> 8),
                          (uint8_t)data};
    return i2c_master_transmit(*dev_handle, data_wr, 3, 1000);
}