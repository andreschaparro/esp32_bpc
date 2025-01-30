#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ads111x.h"
#include "mcp4725.h"
#include "pcf8574.h"

// ================================ Public Defines ================================================

#define SDA_GPIO GPIO_NUM_21 // GPIO pin for SDA
#define SCL_GPIO GPIO_NUM_22 // GPIO pin for SCL
#define I2C_PORT I2C_NUM_0   // I2C port number

// ================================ Public Constants ==============================================

static const char *TAG = "MAIN";

// ================================ Public Functions Declaration ==================================

static esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
static void ads111x_handle_task(void *pvParameter);
static void mcp4725_handle_task(void *pvParameter);
static void pcf8574_handle_task(void *pvParameter);

// ================================ Program Entry Point ===========================================

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init(I2C_PORT, SDA_GPIO, SCL_GPIO));
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(
        ads111x_handle_task,
        "ads111x_handle_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        5,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ads111x_handle_task");
    }
    xReturned = xTaskCreatePinnedToCore(
        mcp4725_handle_task,
        "mcp4725_handle_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        5,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create mcp4725_handle_task");
    }
    xReturned = xTaskCreatePinnedToCore(
        pcf8574_handle_task,
        "pcf8574_handle_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        5,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create pcf8574_handle_task");
    }
    vTaskDelete(NULL);
}

// ================================ Public Functions Definitions ==================================

// Initialize the I2C master bus
static esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    return i2c_new_master_bus(&i2c_mst_config, &bus_handle);
}

// Task to handle the ADS111x device
static void ads111x_handle_task(void *pvParameter)
{
    ads111x_dev_t dev = {0};
    ESP_ERROR_CHECK(ads111x_init(&dev, I2C_PORT, ADS111X_I2C_ADDR_GND));
    ESP_ERROR_CHECK(ads111x_set_mux(&dev, ADS111X_MUX_SINGLE_0));
    ESP_ERROR_CHECK(ads111x_set_pga(&dev, ADS111X_PGA_2_048V));
    ESP_ERROR_CHECK(ads111x_set_mode(&dev, ADS111X_MODE_SINGLE_SHOT));
    ESP_ERROR_CHECK(ads111x_set_dr(&dev, ADS111X_DR_128SPS));
    const ads111x_mux_t mux_channels[] = {
        ADS111X_MUX_SINGLE_0,
        ADS111X_MUX_SINGLE_1,
        ADS111X_MUX_SINGLE_2,
        ADS111X_MUX_SINGLE_3,
    };
    const size_t num_channels = sizeof(mux_channels) / sizeof(mux_channels[0]);
    for (;;)
    {
        for (size_t i = 0; i < num_channels; i++)
        {
            if (ads111x_set_mux(&dev, mux_channels[i]) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to set input multiplexer for input %d", i);
                continue;
            }
            if (ads111x_start_conv(&dev) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start conversion for input %d", i);
                continue;
            }
            bool busy = true;
            while (busy)
            {
                if (ads111x_busy(&dev, &busy) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to check busy status for input %d", i);
                    break;
                }
            }
            if (!busy)
            {
                ads111x_conv_t res = {.raw = 0, .volt = 0.0f};
                if (ads111x_get_conv(&dev, &res) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Input %d: raw=%d, volt=%.3f mV", i, res.raw, res.volt);
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to get conversion result for input %d", i);
                }
            }
            else
            {
                ESP_LOGE(TAG, "Conversion did not complete for input %d", i);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// Task to handle the MCP4725 device
static void mcp4725_handle_task(void *pvParameter)
{
    mcp4725_dev_t dev = {0};
    ESP_ERROR_CHECK(mcp4725_init(&dev, I2C_PORT, MCP4725_I2C_ADDR_GND));
    mcp4725_pd_mode_t pd_mode_eeprom = 0;
    ESP_ERROR_CHECK(mcp4725_get_pd_mode(&dev, &pd_mode_eeprom, true));
    if (pd_mode_eeprom != MCP4725_PD_MODE_NORMAL)
    {
        ESP_ERROR_CHECK(mcp4725_set_pd_mode(&dev, MCP4725_PD_MODE_NORMAL, true));
        bool busy = true;
        while (busy)
        {
            ESP_ERROR_CHECK(mcp4725_eeprom_busy(&dev, &busy));
        }
    }
    uint16_t val_eeprom = 0;
    ESP_ERROR_CHECK(mcp4725_get_raw(&dev, &val_eeprom, true));
    if (val_eeprom != 0)
    {
        ESP_ERROR_CHECK(mcp4725_set_raw(&dev, 0, true));
        bool busy = true;
        while (busy)
        {
            ESP_ERROR_CHECK(mcp4725_eeprom_busy(&dev, &busy));
        }
    }
    uint16_t val = 0;
    for (;;)
    {
        ESP_ERROR_CHECK(mcp4725_set_raw(&dev, val, false));
        if (val <= MCP4725_MAX_VALUE)
        {
            val += 20;
        }
        else
        {
            val = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task to handle the PCF8574 device
static void pcf8574_handle_task(void *pvParameter)
{
    pcf8574_dev_t dev = {0};
    ESP_ERROR_CHECK(pcf8574_init(&dev, I2C_PORT, PCF8574_I2C_ADDR_HHH));
    pcf8574_pin_t pins[] = {
        PCF8574_P0,
        PCF8574_P1,
        PCF8574_P2,
        PCF8574_P3,
        PCF8574_P4,
        PCF8574_P5,
        PCF8574_P6,
        PCF8574_P7,
    };
    size_t num_pins = sizeof(pins) / sizeof(pins[0]);
    pcf8574_state_t val = 0;
    for (;;)
    {
        for (size_t i = 0; i < num_pins; i++)
        {
            ESP_ERROR_CHECK(pcf8574_toogle_pin(&dev, pins[i]));
            ESP_ERROR_CHECK(pcf8574_get_pin(&dev, pins[i], &val));
            ESP_LOGI(TAG, "Pin %d: %s", pins[i], val == PCF8574_LOW ? "LOW" : "HIGH");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}