#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ADS111X_ADDR_GND = 0x48,
        ADS111X_ADDR_VDD,
        ADS111X_ADDR_SDA,
        ADS111X_ADDR_SCL
    } ads111x_address_t;

    typedef enum
    {
        ADS111X_CONV_REG = 0,
        ADS111X_CONFIG_REG,
        ADS111X_LO_THRESH_REG,
        ADS111X_HI_THRESH_REG
    } ads111x_reg_t;

    typedef enum
    {
        ADS111X_OS_BUSY = 0,
        ADS111X_OS_READY
    } ads111x_os_t;

    typedef enum
    {
        ADS111X_MUX_AIN0_AIN1 = 0,
        ADS111X_MUX_AIN0_AIN3,
        ADS111X_MUX_AIN1_AIN3,
        ADS111X_MUX_AIN2_AIN3,
        ADS111X_MUX_AIN0_GND,
        ADS111X_MUX_AIN1_GND,
        ADS111X_MUX_AIN2_GND,
        ADS111X_MUX_AIN3_GND
    } ads111x_mux_t;

    typedef enum
    {
        ADS111X_PGA_6_144V = 0,
        ADS111X_PGA_4_096V,
        ADS111X_PGA_2_048V,
        ADS111X_PGA_1_024V,
        ADS111X_PGA_0_512V,
        ADS111X_PGA_0_256V
    } ads111x_pga_t;

    typedef enum
    {
        ADS111X_MODE_CONTINUOUS = 0,
        ADS111X_MODE_SINGLE_SHOT
    } ads111x_mode_t;

    typedef enum
    {
        ADS111X_DR_8SPS = 0,
        ADS111X_DR_16SPS,
        ADS111X_DR_32SPS,
        ADS111X_DR_64SPS,
        ADS111X_DR_128SPS,
        ADS111X_DR_250SPS,
        ADS111X_DR_475SPS,
        ADS111X_DR_860SPS
    } ads111x_dr_t;

    typedef enum
    {
        ADS111X_COMP_MODE_TRADITIONAL = 0,
        ADS111X_COMP_MODE_WINDOW
    } ads111x_comp_mode_t;

    typedef enum
    {
        ADS111X_COMP_POL_LOW = 0,
        ADS111X_COMP_POL_HIGH
    } ads111x_comp_pol_t;

    typedef enum
    {
        ADS111X_COMP_NON_LATCH = 0,
        ADS111X_COMP_LATCH
    } ads111x_comp_lat_t;

    typedef enum
    {
        ADS111X_COMP_QUE_1 = 0,
        ADS111X_COMP_QUE_2,
        ADS111X_COMP_QUE_4,
        ADS111X_COMP_QUE_DISABLE
    } ads111x_comp_que_t;

    esp_err_t ads111x_init(i2c_master_dev_handle_t *dev_handle,
                           i2c_port_num_t port,
                           ads111x_address_t address,
                           uint32_t speed_hz);
    esp_err_t ads111x_read_os(i2c_master_dev_handle_t *dev_handle,
                              ads111x_os_t *os);
    esp_err_t ads111x_start_single_conv(i2c_master_dev_handle_t *dev_handle);
    esp_err_t ads111x_read_mux(i2c_master_dev_handle_t *dev_handle,
                               ads111x_mux_t *mux);
    esp_err_t ads111x_write_mux(i2c_master_dev_handle_t *dev_handle,
                                ads111x_mux_t mux);
    esp_err_t ads111x_read_pga(i2c_master_dev_handle_t *dev_handle,
                               ads111x_pga_t *pga);
    esp_err_t ads111x_write_pga(i2c_master_dev_handle_t *dev_handle,
                                ads111x_pga_t pga);
    esp_err_t ads111x_read_mode(i2c_master_dev_handle_t *dev_handle,
                                ads111x_mode_t *mode);
    esp_err_t ads111x_write_mode(i2c_master_dev_handle_t *dev_handle,
                                 ads111x_mode_t mode);
    esp_err_t ads111x_read_dr(i2c_master_dev_handle_t *dev_handle,
                              ads111x_dr_t *dr);
    esp_err_t ads111x_write_dr(i2c_master_dev_handle_t *dev_handle,
                               ads111x_dr_t dr);
    esp_err_t ads111x_read_comp_mode(i2c_master_dev_handle_t *dev_handle,
                                     ads111x_comp_mode_t *comp_mode);
    esp_err_t ads111x_write_comp_mode(i2c_master_dev_handle_t *dev_handle,
                                      ads111x_comp_mode_t comp_mode);
    esp_err_t ads111x_read_comp_pol(i2c_master_dev_handle_t *dev_handle,
                                    ads111x_comp_pol_t *comp_pol);
    esp_err_t ads111x_write_comp_pol(i2c_master_dev_handle_t *dev_handle,
                                     ads111x_comp_pol_t comp_pol);
    esp_err_t ads111x_read_comp_lat(i2c_master_dev_handle_t *dev_handle,
                                    ads111x_comp_lat_t *comp_lat);
    esp_err_t ads111x_write_comp_lat(i2c_master_dev_handle_t *dev_handle,
                                     ads111x_comp_lat_t comp_lat);
    esp_err_t ads111x_read_comp_que(i2c_master_dev_handle_t *dev_handle,
                                    ads111x_comp_que_t *comp_que);
    esp_err_t ads111x_write_comp_que(i2c_master_dev_handle_t *dev_handle,
                                     ads111x_comp_que_t comp_que);
    esp_err_t ads111x_read_comp_lo_thresh(i2c_master_dev_handle_t *dev_handle,
                                          int16_t *val);
    esp_err_t ads111x_write_comp_lo_thresh(i2c_master_dev_handle_t *dev_handle,
                                           int16_t val);
    esp_err_t ads111x_read_comp_hi_thresh(i2c_master_dev_handle_t *dev_handle,
                                          int16_t *val);
    esp_err_t ads111x_write_comp_hi_thresh(i2c_master_dev_handle_t *dev_handle,
                                           int16_t val);
    esp_err_t ads111x_read_conv(i2c_master_dev_handle_t *dev_handle,
                                int16_t *val);
    esp_err_t ads111x_read_lsb_size(i2c_master_dev_handle_t *dev_handle,
                                    float *val);

#ifdef __cplusplus
}
#endif
