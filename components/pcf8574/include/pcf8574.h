#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        PCF8574_ADDR_LLL = 0x20,
        PCF8574_ADDR_LLH,
        PCF8574_ADDR_LHL,
        PCF8574_ADDR_LHH,
        PCF8574_ADDR_HLL,
        PCF8574_ADDR_HLH,
        PCF8574_ADDR_HHL,
        PCF8574_ADDR_HHH
    } pcf8574_address_t;

    typedef enum
    {
        PCF8574_PIN_P0 = 0,
        PCF8574_PIN_P1,
        PCF8574_PIN_P2,
        PCF8574_PIN_P3,
        PCF8574_PIN_P4,
        PCF8574_PIN_P5,
        PCF8574_PIN_P6,
        PCF8574_PIN_P7
    } pcf8574_pin_t;

    typedef enum
    {
        PCF8574_LOW = 0,
        PCF8574_HIGH
    } pcf8574_level_t;

    typedef struct
    {
        i2c_master_dev_handle_t dev_handle;
        uint8_t port_status;
    } pcf8574_device_t;

    esp_err_t pcf8574_init(pcf8574_device_t *device,
                           i2c_port_t port,
                           pcf8574_address_t address,
                           uint32_t speed_hz);
    esp_err_t pcf8574_read_pin(pcf8574_device_t *device,
                               pcf8574_pin_t pin,
                               pcf8574_level_t *level);
    esp_err_t pcf8574_write_pin(pcf8574_device_t *device,
                                pcf8574_pin_t pin,
                                pcf8574_level_t level);
    esp_err_t pcf8574_toggle_pin(pcf8574_device_t *device,
                                 pcf8574_pin_t pin);

#ifdef __cplusplus
}
#endif
