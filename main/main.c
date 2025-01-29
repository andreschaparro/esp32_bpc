#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ads111x.h"
#include "mcp4725.h"

// ================================ Public Defines ================================================

#define SDA_GPIO GPIO_NUM_21 // GPIO pin for SDA
#define SCL_GPIO GPIO_NUM_22 // GPIO pin for SCL
#define I2C_PORT I2C_NUM_0   // I2C port number

// ================================ Public Constants ==============================================

static const char *TAG = "MAIN";

// ================================ Public Functions Declaration ==================================

static esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
static void ads111x_test_task(void *pvParameter);
static void mcp4725_test_task(void *pvParameter);

// ================================ Program Entry Point ===========================================

void app_main(void)
{
    esp_err_t ret = i2c_master_init(I2C_PORT, SDA_GPIO, SCL_GPIO); // Initialize the I2C master bus
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus");
        esp_restart();
    }
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore( // Create a task to test the ADS111x device
        ads111x_test_task,
        "ads111x_test_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        5,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ads111x_test_task");
    }
    xReturned = xTaskCreatePinnedToCore( // Create a task to test the MCP4725 device
        mcp4725_test_task,
        "mcp4725_test_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        5,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create mcp4725_test_task");
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

// Task to test the ADS111x device
static void ads111x_test_task(void *pvParameter)
{
    ads111x_dev_t dev;
    memset(&dev, 0, sizeof(ads111x_dev_t));
    if (ads111x_init(&dev, I2C_PORT, ADS111X_I2C_ADDR_GND, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize device");
        vTaskDelete(NULL);
    }
    bool busy;
    if (ads111x_busy(&dev, &busy) == ESP_OK)
    {
        ESP_LOGI(TAG, "Device is %s", busy ? "busy" : "ready");
    }
    if (ads111x_set_mux(&dev, ADS111X_MUX_SINGLE_0) == ESP_OK)
    {
        ads111x_mux_t mux = 0;
        if (ads111x_get_mux(&dev, &mux) == ESP_OK)
        {
            ESP_LOGI(TAG, "Input multiplexer: %d", mux);
        }
    }

    if (ads111x_set_pga(&dev, ADS111X_PGA_4_096V) == ESP_OK)
    {
        ads111x_pga_t pga = 0;
        if (ads111x_get_pga(&dev, &pga) == ESP_OK)
        {
            ESP_LOGI(TAG, "Programmable gain amplifier: %d", pga);
        }
    }
    if (ads111x_set_mode(&dev, ADS111X_MODE_CONTINUOUS) == ESP_OK)
    {
        ads111x_mode_t mode = 0;
        if (ads111x_get_mode(&dev, &mode) == ESP_OK)
        {
            ESP_LOGI(TAG, "Operating mode: %d", mode);
        }
    }
    if (ads111x_set_dr(&dev, ADS111X_DR_250SPS) == ESP_OK)
    {
        ads111x_dr_t dr = 0;
        if (ads111x_get_dr(&dev, &dr) == ESP_OK)
        {
            ESP_LOGI(TAG, "Data rate: %d", dr);
        }
    }

    if (ads111x_set_comp_mode(&dev, ADS111X_COMP_MODE_WINDOW) == ESP_OK)
    {
        ads111x_comp_mode_t comp = 0;
        if (ads111x_get_comp_mode(&dev, &comp) == ESP_OK)
        {
            ESP_LOGI(TAG, "Comparator mode: %d", comp);
        }
    }
    if (ads111x_set_comp_pol(&dev, ADS111X_COMP_POL_HIGH) == ESP_OK)
    {
        ads111x_comp_pol_t pol = 0;
        if (ads111x_get_comp_pol(&dev, &pol) == ESP_OK)
        {
            ESP_LOGI(TAG, "Comparator polarity: %d", pol);
        }
    }
    if (ads111x_set_comp_lat(&dev, ADS111X_COMP_LATCH) == ESP_OK)
    {
        ads111x_comp_lat_t lat = 0;
        if (ads111x_get_comp_lat(&dev, &lat) == ESP_OK)
        {
            ESP_LOGI(TAG, "Comparator latching: %d", lat);
        }
    }
    if (ads111x_set_comp_que(&dev, ADS111X_COMP_QUE_4) == ESP_OK)
    {
        ads111x_comp_que_t que = 0;
        if (ads111x_get_comp_que(&dev, &que) == ESP_OK)
        {
            ESP_LOGI(TAG, "Comparator queue: %d", que);
        }
    }
    if (ads111x_set_comp_lo_thresh(&dev, 1000) == ESP_OK)
    {
        int16_t lo_thresh = 0;
        if (ads111x_get_comp_lo_thresh(&dev, &lo_thresh) == ESP_OK)
        {
            ESP_LOGI(TAG, "Comparator low threshold: %d", lo_thresh);
        }
    }
    if (ads111x_set_comp_hi_thresh(&dev, 2000) == ESP_OK)
    {
        int16_t hi_thresh = 0;
        if (ads111x_get_comp_hi_thresh(&dev, &hi_thresh) == ESP_OK)
        {
            ESP_LOGI(TAG, "Comparator high threshold: %d", hi_thresh);
        }
    }
    for (;;)
    {
        ads111x_conv_t res = {.raw = 0, .volt = 0.0f};
        if (ads111x_get_conv(&dev, &res) == ESP_OK)
        {
            ESP_LOGI(TAG, "Conversion result: raw=%d , volt=%.3f mV", res.raw, res.volt);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task to test the MCP4725 device
static void mcp4725_test_task(void *pvParameter)
{
    mcp4725_dev_t dev;
    memset(&dev, 0, sizeof(mcp4725_dev_t));
    if (mcp4725_init(&dev, I2C_PORT, MCP4725_I2C_ADDR_GND, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize device");
        vTaskDelete(NULL);
    }
    bool busy;
    if (mcp4725_eeprom_busy(&dev, &busy) == ESP_OK)
    {
        ESP_LOGI(TAG, "EEPROM is %s", busy ? "busy" : "ready");
    }
    mcp4725_pd_mode_t pd = 0;
    if (mcp4725_get_pd_mode(&dev, true, &pd) == ESP_OK)
    {
        ESP_LOGI(TAG, "Power-down mode: %d", pd);
    }
    uint16_t val = 0;
    if (mcp4725_get_raw(&dev, true, &val) == ESP_OK)
    {
        ESP_LOGI(TAG, "Raw value: %d", val);
    }
    if (mcp4725_get_raw(&dev, false, &val) == ESP_OK)
    {
        ESP_LOGI(TAG, "Raw value: %d", val);
    }
    float volt = 0.0f;
    if (mcp4725_get_volt(&dev, true, 3.3f, &volt) == ESP_OK)
    {
        ESP_LOGI(TAG, "Voltage: %.3f V", volt);
    }
    if (mcp4725_get_volt(&dev, false, 3.3f, &volt) == ESP_OK)
    {
        ESP_LOGI(TAG, "Voltage: %.3f V", volt);
    }
    vTaskDelete(NULL);
}