#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "ads111x.h"

// ================================ Private Defines ===============================================

// I2C master configuration
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define I2C_TIMEOUT_MS 1000       // I2C timeout in milliseconds
// ADS111X Registers
#define ADS111X_REG_CONV 0x00      // Conversion register
#define ADS111X_REG_CONFIG 0x01    // Configuration register
#define ADS111X_REG_LO_THRESH 0x02 // Low threshold register
#define ADS111X_REG_HI_THRESH 0x03 // High threshold register
// ADS111X I2C Addresses
#define ADS111X_I2C_ADDR_GND 0x48 // ADDR pin connected to GND
#define ADS111X_I2C_ADDR_VDD 0x49 // ADDR pin connected to VDD
#define ADS111X_I2C_ADDR_SDA 0x4A // ADDR pin connected to SDA
#define ADS111X_I2C_ADDR_SCL 0x4B // ADDR pin connected to SCL
// ADS111X Operational Status
#define ADS111X_OS_BUSY 0   // Conversion in progress
#define ADS111X_OS_SINGLE 1 // Start a single conversion
// ADS111X MASKS and OFFSETS
#define ADS111X_OS_MASK 0x01        // Operational status mask
#define ADS111X_OS_OFFSET 15        // Operational status offset
#define ADS111X_MUX_MASK 0x07       // Input multiplexer configuration mask
#define ADS111X_MUX_OFFSET 12       // Input multiplexer configuration offset
#define ADS111X_PGA_MASK 0x07       // Programmable gain amplifier configuration mask
#define ADS111X_PGA_OFFSET 9        // Programmable gain amplifier configuration offset
#define ADS111X_MODE_MASK 0x01      // Device operating mode mask
#define ADS111X_MODE_OFFSET 8       // Device operating mode offset
#define ADS111X_DR_MASK 0x07        // Data rate mask
#define ADS111X_DR_OFFSET 5         // Data rate offset
#define ADS111X_COMP_MODE_MASK 0x01 // Comparator mode mask
#define ADS111X_COMP_MODE_OFFSET 4  // Comparator mode offset
#define ADS111X_COMP_POL_MASK 0x01  // Comparator polarity mask
#define ADS111X_COMP_POL_OFFSET 3   // Comparator polarity offset
#define ADS111X_COMP_LAT_MASK 0x01  // Comparator latching mask
#define ADS111X_COMP_LAT_OFFSET 2   // Comparator latching offset
#define ADS111X_COMP_QUE_MASK 0x03  // Comparator queue mask
#define ADS111X_COMP_QUE_OFFSET 0   // Comparator queue offset

// ================================ Private Functions Declarations ================================

static esp_err_t ads111x_reg_fld_wr(ads111x_dev_t *dev, uint8_t addr, uint16_t val, uint16_t mask, uint8_t off);
static esp_err_t ads111x_reg_fld_rd(ads111x_dev_t *dev, uint8_t addr, uint16_t *val, uint16_t mask, uint8_t off);
static esp_err_t ads111x_reg_val_rd(ads111x_dev_t *dev, uint8_t addr, uint16_t *val);
static esp_err_t ads111x_reg_val_wr(ads111x_dev_t *dev, uint8_t addr, uint16_t val);

// ================================ Public Functions Definitions ==================================

// Initialize the device
esp_err_t ads111x_init(ads111x_dev_t *dev, i2c_port_num_t port, uint16_t addr, gpio_num_t sda, gpio_num_t scl)
{
    if (dev == NULL ||
        port >= I2C_NUM_MAX ||
        addr < ADS111X_I2C_ADDR_GND || addr > ADS111X_I2C_ADDR_SCL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    dev->i2c_port = port;
    dev->i2c_addr = addr;
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_master_get_bus_handle(dev->i2c_port, &bus_handle);
    if (ret == ESP_ERR_INVALID_STATE)
    {
        i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = port,
            .scl_io_num = scl,
            .sda_io_num = sda,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }
    else if (ret != ESP_OK)
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

// Check if the device is busy
esp_err_t ads111x_busy(ads111x_dev_t *dev, bool *busy)
{
    if (dev == NULL || busy == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t aux = 0;
    esp_err_t ret = ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, &aux, ADS111X_OS_MASK, ADS111X_OS_OFFSET);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *busy = (aux == ADS111X_OS_BUSY);
    return ESP_OK;
}

// Start a single conversion
esp_err_t ads111x_start_conv(ads111x_dev_t *dev)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, ADS111X_OS_SINGLE, ADS111X_OS_MASK, ADS111X_OS_OFFSET);
}

// Get the input multiplexer
esp_err_t ads111x_get_mux(ads111x_dev_t *dev, ads111x_mux_t *mux)
{
    if (dev == NULL || mux == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)mux, ADS111X_MUX_MASK, ADS111X_MUX_OFFSET);
}

// Set the input multiplexer
esp_err_t ads111x_set_mux(ads111x_dev_t *dev, ads111x_mux_t mux)
{
    if (dev == NULL ||
        mux < ADS111X_MUX_DIFF_0_1 || mux > ADS111X_MUX_SINGLE_3)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)mux, ADS111X_MUX_MASK, ADS111X_MUX_OFFSET);
}

// Get the programmable gain amplifier
esp_err_t ads111x_get_pga(ads111x_dev_t *dev, ads111x_pga_t *pga)
{
    if (dev == NULL || pga == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)pga, ADS111X_PGA_MASK, ADS111X_PGA_OFFSET);
}

// Set the programmable gain amplifier
esp_err_t ads111x_set_pga(ads111x_dev_t *dev, ads111x_pga_t pga)
{
    if (dev == NULL ||
        pga < ADS111X_PGA_6_144V || pga > ADS111X_PGA_0_256V)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)pga, ADS111X_PGA_MASK, ADS111X_PGA_OFFSET);
}

// Get the operating mode
esp_err_t ads111x_get_mode(ads111x_dev_t *dev, ads111x_mode_t *mode)
{
    if (dev == NULL || mode == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)mode, ADS111X_MODE_MASK, ADS111X_MODE_OFFSET);
}

// Set the operating mode
esp_err_t ads111x_set_mode(ads111x_dev_t *dev, ads111x_mode_t mode)
{
    if (dev == NULL ||
        mode < ADS111X_MODE_CONTINUOUS || mode > ADS111X_MODE_SINGLE_SHOT)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)mode, ADS111X_MODE_MASK, ADS111X_MODE_OFFSET);
}

// Get the data rate
esp_err_t ads111x_get_dr(ads111x_dev_t *dev, ads111x_dr_t *rate)
{
    if (dev == NULL || rate == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)rate, ADS111X_DR_MASK, ADS111X_DR_OFFSET);
}

// Set the data rate
esp_err_t ads111x_set_dr(ads111x_dev_t *dev, ads111x_dr_t rate)
{
    if (dev == NULL ||
        rate < ADS111X_DR_8SPS || rate > ADS111X_DR_860SPS)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)rate, ADS111X_DR_MASK, ADS111X_DR_OFFSET);
}

// Get the comparator mode
esp_err_t ads111x_get_comp_mode(ads111x_dev_t *dev, ads111x_comp_mode_t *comp)
{
    if (dev == NULL || comp == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)comp, ADS111X_COMP_MODE_MASK, ADS111X_COMP_MODE_OFFSET);
}

// Set the comparator mode
esp_err_t ads111x_set_comp_mode(ads111x_dev_t *dev, ads111x_comp_mode_t comp)
{
    if (dev == NULL ||
        comp < ADS111X_COMP_MODE_TRADITIONAL || comp > ADS111X_COMP_MODE_WINDOW)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)comp, ADS111X_COMP_MODE_MASK, ADS111X_COMP_MODE_OFFSET);
}

// Get the comparator polarity
esp_err_t ads111x_get_comp_pol(ads111x_dev_t *dev, ads111x_comp_pol_t *pol)
{
    if (dev == NULL || pol == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)pol, ADS111X_COMP_POL_MASK, ADS111X_COMP_POL_OFFSET);
}

// Set the comparator polarity
esp_err_t ads111x_set_comp_pol(ads111x_dev_t *dev, ads111x_comp_pol_t pol)
{
    if (dev == NULL ||
        pol < ADS111X_COMP_POL_LOW || pol > ADS111X_COMP_POL_HIGH)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)pol, ADS111X_COMP_POL_MASK, ADS111X_COMP_POL_OFFSET);
}

// Get the comparator latching
esp_err_t ads111x_get_comp_lat(ads111x_dev_t *dev, ads111x_comp_lat_t *lat)
{
    if (dev == NULL || lat == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)lat, ADS111X_COMP_LAT_MASK, ADS111X_COMP_LAT_OFFSET);
}

// Set the comparator latching
esp_err_t ads111x_set_comp_lat(ads111x_dev_t *dev, ads111x_comp_lat_t lat)
{
    if (dev == NULL ||
        lat < ADS111X_COMP_NON_LAT || lat > ADS111X_COMP_LAT)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)lat, ADS111X_COMP_LAT_MASK, ADS111X_COMP_LAT_OFFSET);
}

// Get the comparator queue
esp_err_t ads111x_get_comp_que(ads111x_dev_t *dev, ads111x_comp_que_t *que)
{
    if (dev == NULL || que == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_rd(dev, ADS111X_REG_CONFIG, (uint16_t *)que, ADS111X_COMP_QUE_MASK, ADS111X_COMP_QUE_OFFSET);
}

// Set the comparator queue
esp_err_t ads111x_set_comp_que(ads111x_dev_t *dev, ads111x_comp_que_t que)
{
    if (dev == NULL ||
        que < ADS111X_COMP_QUE_1 || que > ADS111X_COMP_QUE_DISABLE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_fld_wr(dev, ADS111X_REG_CONFIG, (uint16_t)que, ADS111X_COMP_QUE_MASK, ADS111X_COMP_QUE_OFFSET);
}

// Get the low threshold comparator
esp_err_t ads111x_get_comp_lo_thresh(ads111x_dev_t *dev, int16_t *val)
{
    if (dev == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_val_rd(dev, ADS111X_REG_LO_THRESH, (uint16_t *)val);
}

// Set the low threshold comparator
esp_err_t ads111x_set_comp_lo_thresh(ads111x_dev_t *dev, int16_t val)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_val_wr(dev, ADS111X_REG_LO_THRESH, (uint16_t)val);
}

// Get the high threshold comparator
esp_err_t ads111x_get_comp_hi_thresh(ads111x_dev_t *dev, int16_t *val)
{
    if (dev == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_val_rd(dev, ADS111X_REG_HI_THRESH, (uint16_t *)val);
}

// Set the high threshold comparator
esp_err_t ads111x_set_comp_hi_thresh(ads111x_dev_t *dev, int16_t val)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_val_wr(dev, ADS111X_REG_HI_THRESH, (uint16_t)val);
}

// Get the last conversion result
esp_err_t ads111x_get_conv(ads111x_dev_t *dev, int16_t *val)
{
    if (dev == NULL || val == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ads111x_reg_val_rd(dev, ADS111X_REG_CONV, (uint16_t *)val);
}

// ================================ Private Functions Definitions =================================

// Low level function to write a register field
static esp_err_t ads111x_reg_fld_wr(ads111x_dev_t *dev, uint8_t addr, uint16_t val, uint16_t mask, uint8_t off)
{
    uint16_t aux = 0;
    esp_err_t ret = ads111x_reg_val_rd(dev, addr, &aux);
    if (ret != ESP_OK)
    {
        return ret;
    }
    aux = (aux & ~(mask << off)) | ((val << off) & (mask << off));
    return ads111x_reg_val_wr(dev, addr, aux);
}

// Low level function to read a register field
static esp_err_t ads111x_reg_fld_rd(ads111x_dev_t *dev, uint8_t addr, uint16_t *val, uint16_t mask, uint8_t off)
{
    esp_err_t ret = ads111x_reg_val_rd(dev, addr, val);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ESP_LOGI("ads111x", "val: %d", *val);
    *val = (*val & (mask << off)) >> off;
    return ESP_OK;
}

// Low level function to read a register value
static esp_err_t ads111x_reg_val_rd(ads111x_dev_t *dev, uint8_t addr, uint16_t *val)
{
    uint8_t data_wr = addr;
    uint8_t data_rd[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &data_wr, sizeof(data_wr), data_rd, sizeof(data_rd), I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }
    *val = (data_rd[0] << 8) | data_rd[1]; // MSB | LSB
    return ESP_OK;
}

// Low level function to write a register value
static esp_err_t ads111x_reg_val_wr(ads111x_dev_t *dev, uint8_t addr, uint16_t val)
{
    uint8_t data_wr[3] = {0};
    data_wr[0] = addr;              // Register address
    data_wr[1] = (val >> 8) & 0xFF; // MSB
    data_wr[2] = val & 0xFF;        // LSB
    return i2c_master_transmit(dev->i2c_dev, data_wr, 3, I2C_TIMEOUT_MS);
}