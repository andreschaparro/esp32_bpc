#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ================================ Public Defines ============================================

#define ADS111X_I2C_ADDR_GND 0x48 // I2C address with ADDR pin connected to GND
#define ADS111X_I2C_ADDR_VDD 0x49 // I2C address with ADDR pin connected to VDD
#define ADS111X_I2C_ADDR_SDA 0x4A // I2C address with ADDR pin connected to SDA
#define ADS111X_I2C_ADDR_SCL 0x4B // I2C address with ADDR pin connected to SCL

    // ================================ Public Data Types =========================================

    typedef struct
    {
        i2c_master_dev_handle_t i2c_dev;
        i2c_port_t i2c_port;
        uint16_t i2c_addr;
    } ads111x_dev_t;

    typedef enum
    {
        ADS111X_MUX_DIFF_0_1 = 0, // Differential P = AIN0, N = AIN1 (default)
        ADS111X_MUX_DIFF_0_3,     // Differential P = AIN0, N = AIN3
        ADS111X_MUX_DIFF_1_3,     // Differential P = AIN1, N = AIN3
        ADS111X_MUX_DIFF_2_3,     // Differential P = AIN2, N = AIN3
        ADS111X_MUX_SINGLE_0,     // Single-ended AIN0
        ADS111X_MUX_SINGLE_1,     // Single-ended AIN1
        ADS111X_MUX_SINGLE_2,     // Single-ended AIN2
        ADS111X_MUX_SINGLE_3,     // Single-ended AIN3
    } ads111x_mux_t;

    typedef enum
    {
        ADS111X_PGA_6_144V = 0, // +/-6.144V range = Gain 2/3
        ADS111X_PGA_4_096V,     // +/-4.096V range = Gain 1
        ADS111X_PGA_2_048V,     // +/-2.048V range = Gain 2 (default)
        ADS111X_PGA_1_024V,     // +/-1.024V range = Gain 4
        ADS111X_PGA_0_512V,     // +/-0.512V range = Gain 8
        ADS111X_PGA_0_256V,     // +/-0.256V range = Gain 16
    } ads111x_pga_t;

    typedef enum
    {
        ADS111X_MODE_CONTINUOUS = 0, // Continuous conversion mode
        ADS111X_MODE_SINGLE_SHOT,    // Power-down single-shot mode (default)
    } ads111x_mode_t;

    typedef enum
    {
        ADS111X_DR_8SPS = 0, // 8 samples per second
        ADS111X_DR_16SPS,    // 16 samples per second
        ADS111X_DR_32SPS,    // 32 samples per second
        ADS111X_DR_64SPS,    // 64 samples per second
        ADS111X_DR_128SPS,   // 128 samples per second (default)
        ADS111X_DR_250SPS,   // 250 samples per second
        ADS111X_DR_475SPS,   // 475 samples per second
        ADS111X_DR_860SPS,   // 860 samples per second
    } ads111x_dr_t;

    typedef enum
    {
        ADS111X_COMP_MODE_TRADITIONAL = 0, // Traditional comparator with hysteresis (default)
        ADS111X_COMP_MODE_WINDOW,          // Window comparator
    } ads111x_comp_mode_t;

    typedef enum
    {
        ADS111X_COMP_POL_LOW = 0, // Active low (default)
        ADS111X_COMP_POL_HIGH,    // Active high
    } ads111x_comp_pol_t;

    typedef enum
    {
        ADS111X_COMP_NON_LAT = 0, // Non-latching comparator (default)
        ADS111X_COMP_LAT,         // Latching comparator
    } ads111x_comp_lat_t;

    typedef enum
    {
        ADS111X_COMP_QUE_1 = 0,  // Assert after one conversion
        ADS111X_COMP_QUE_2,      // Assert after two conversions
        ADS111X_COMP_QUE_4,      // Assert after four conversions
        ADS111X_COMP_QUE_DISABLE // Disable comparator (default)
    } ads111x_comp_que_t;

    // ================================ Public Functions Declaration ==============================

    esp_err_t ads111x_init(ads111x_dev_t *dev, i2c_port_num_t port, uint16_t addr, gpio_num_t sda, gpio_num_t scl);
    esp_err_t ads111x_busy(ads111x_dev_t *dev, bool *busy);
    esp_err_t ads111x_start_conv(ads111x_dev_t *dev);
    esp_err_t ads111x_get_mux(ads111x_dev_t *dev, ads111x_mux_t *mux);
    esp_err_t ads111x_set_mux(ads111x_dev_t *dev, ads111x_mux_t mux);
    esp_err_t ads111x_get_pga(ads111x_dev_t *dev, ads111x_pga_t *pga);
    esp_err_t ads111x_set_pga(ads111x_dev_t *dev, ads111x_pga_t pga);
    esp_err_t ads111x_get_mode(ads111x_dev_t *dev, ads111x_mode_t *mode);
    esp_err_t ads111x_set_mode(ads111x_dev_t *dev, ads111x_mode_t mode);
    esp_err_t ads111x_get_dr(ads111x_dev_t *dev, ads111x_dr_t *rate);
    esp_err_t ads111x_set_dr(ads111x_dev_t *dev, ads111x_dr_t rate);
    esp_err_t ads111x_get_comp_mode(ads111x_dev_t *dev, ads111x_comp_mode_t *comp);
    esp_err_t ads111x_set_comp_mode(ads111x_dev_t *dev, ads111x_comp_mode_t comp);
    esp_err_t ads111x_get_comp_pol(ads111x_dev_t *dev, ads111x_comp_pol_t *pol);
    esp_err_t ads111x_set_comp_pol(ads111x_dev_t *dev, ads111x_comp_pol_t pol);
    esp_err_t ads111x_get_comp_lat(ads111x_dev_t *dev, ads111x_comp_lat_t *lat);
    esp_err_t ads111x_set_comp_lat(ads111x_dev_t *dev, ads111x_comp_lat_t lat);
    esp_err_t ads111x_get_comp_que(ads111x_dev_t *dev, ads111x_comp_que_t *que);
    esp_err_t ads111x_set_comp_que(ads111x_dev_t *dev, ads111x_comp_que_t que);
    esp_err_t ads111x_get_comp_lo_thresh(ads111x_dev_t *dev, int16_t *val);
    esp_err_t ads111x_set_comp_lo_thresh(ads111x_dev_t *dev, int16_t val);
    esp_err_t ads111x_get_comp_hi_thresh(ads111x_dev_t *dev, int16_t *val);
    esp_err_t ads111x_set_comp_hi_thresh(ads111x_dev_t *dev, int16_t val);
    esp_err_t ads111x_get_conv(ads111x_dev_t *dev, int16_t *val);

#ifdef __cplusplus
}
#endif