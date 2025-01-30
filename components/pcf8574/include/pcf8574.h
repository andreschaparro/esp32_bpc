#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // ================================ Public Defines ============================================

#define PCF8574_I2C_ADDR_LLL 0x20 // I2C address with A0, A1, A2 pins connected to GND
#define PCF8574_I2C_ADDR_LLH 0x21 // I2C address with A0 pin connected to VDD, A1, A2 pins connected to GND
#define PCF8574_I2C_ADDR_LHL 0x22 // I2C address with A1 pin connected to VDD, A0, A2 pins connected to GND
#define PCF8574_I2C_ADDR_LHH 0x23 // I2C address with A0, A1 pins connected to VDD, A2 pin connected to GND
#define PCF8574_I2C_ADDR_HLL 0x24 // I2C address with A2 pin connected to VDD, A0, A1 pins connected to GND
#define PCF8574_I2C_ADDR_HLH 0x25 // I2C address with A0, A2 pins connected to VDD, A1 pin connected to GND
#define PCF8574_I2C_ADDR_HHL 0x26 // I2C address with A1, A2 pins connected to VDD, A0 pin connected to GND
#define PCF8574_I2C_ADDR_HHH 0x27 // I2C address with A0, A1, A2 pins connected to VDD

    // ================================ Public Data Types =========================================

    typedef struct
    {
        i2c_master_dev_handle_t i2c_dev;
        i2c_port_t i2c_port;
        uint16_t i2c_addr;
    } pcf8574_dev_t;

    typedef enum
    {
        PCF8574_P0 = 0,
        PCF8574_P1,
        PCF8574_P2,
        PCF8574_P3,
        PCF8574_P4,
        PCF8574_P5,
        PCF8574_P6,
        PCF8574_P7,
    } pcf8574_pin_t;

    typedef enum
    {
        PCF8574_OUTPUT = 0,
        PCF8574_INPUT,
    } pcf8574_dir_t;

    typedef enum
    {
        PCF8574_LOW = 0,
        PCF8574_HIGH,
    } pcf8574_state_t;

    // ================================ Public Functions Declaration ==============================

    esp_err_t pcf8574_init(pcf8574_dev_t *dev, i2c_port_t port, uint16_t addr);
    esp_err_t pcf8574_get_pin(pcf8574_dev_t *dev, pcf8574_pin_t pin, pcf8574_state_t *val);
    esp_err_t pcf8574_set_pin(pcf8574_dev_t *dev, pcf8574_pin_t pin, pcf8574_state_t val);
    esp_err_t pcf8574_set_pin_dir(pcf8574_dev_t *dev, pcf8574_pin_t pin, pcf8574_dir_t dir);
    esp_err_t pcf8574_toogle_pin(pcf8574_dev_t *dev, pcf8574_pin_t pin);

#ifdef __cplusplus
}
#endif