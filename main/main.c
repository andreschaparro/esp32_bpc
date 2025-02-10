#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ads111x.h"
#include "mcp4725.h"
#include "pcf8574.h"

#define SDA_GPIO GPIO_NUM_21
#define SCL_GPIO GPIO_NUM_22
#define I2C_PORT I2C_NUM_0
#define I2C_SPEED_HZ 100000

static const char *TAG = "MAIN";

static esp_err_t i2c_master_init(i2c_port_t port,
                                 gpio_num_t sda,
                                 gpio_num_t scl);
static void vTaskADS1115(void *pvParameter);
static void vTaskMCP4725(void *pvParameter);
static void vTaskPCF8574A(void *pvParameter);
static void vTaskPCF8574B(void *pvParameter);

void app_main(void)
{
    if (i2c_master_init(I2C_PORT, SDA_GPIO, SCL_GPIO) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master");
        vTaskDelete(NULL);
    }
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(
        vTaskADS1115,
        "vTaskADS1115",
        configMINIMAL_STACK_SIZE * 10,
        NULL,
        tskIDLE_PRIORITY + 5U,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create vTaskADS111x");
    }
    xReturned = xTaskCreatePinnedToCore(
        vTaskMCP4725,
        "vTaskMCP4725",
        configMINIMAL_STACK_SIZE * 10,
        NULL,
        tskIDLE_PRIORITY + 5U,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create vTaskMCP4725");
    }
    xReturned = xTaskCreatePinnedToCore(
        vTaskPCF8574A,
        "vTaskPCF8574A",
        configMINIMAL_STACK_SIZE * 10,
        NULL,
        tskIDLE_PRIORITY + 5U,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create vTaskPCF8574A");
    }
    xReturned = xTaskCreatePinnedToCore(
        vTaskPCF8574B,
        "vTaskPCF8574B",
        configMINIMAL_STACK_SIZE * 10,
        NULL,
        tskIDLE_PRIORITY + 5U,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create vTaskPCF8574B");
    }
    vTaskDelete(NULL);
}

static esp_err_t i2c_master_init(i2c_port_t port,
                                 gpio_num_t sda,
                                 gpio_num_t scl)
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

static void vTaskADS1115(void *pvParameter)
{
    i2c_master_dev_handle_t dev_handle;
    float lsb_size = 0.0f;
    if (ads111x_init(&dev_handle, I2C_PORT, ADS111X_ADDR_GND, I2C_SPEED_HZ) != ESP_OK ||
        ads111x_write_pga(&dev_handle, ADS111X_PGA_2_048V) != ESP_OK ||
        ads111x_write_mode(&dev_handle, ADS111X_MODE_SINGLE_SHOT) != ESP_OK ||
        ads111x_write_dr(&dev_handle, ADS111X_DR_128SPS) != ESP_OK ||
        ads111x_read_lsb_size(&dev_handle, &lsb_size) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize ADS1115");
        vTaskDelete(NULL);
    }
    lsb_size /= 1000.0f;
    const ads111x_mux_t mux_channels[] = {
        ADS111X_MUX_AIN0_GND,
        ADS111X_MUX_AIN1_GND,
        ADS111X_MUX_AIN2_GND,
        ADS111X_MUX_AIN3_GND};
    const size_t num_channels = sizeof(mux_channels) / sizeof(mux_channels[0]);
    for (;;)
    {
        for (size_t i = 0; i < num_channels; i++)
        {
            if (ads111x_write_mux(&dev_handle, mux_channels[i]) != ESP_OK ||
                ads111x_start_single_conv(&dev_handle) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start single conversion for AIN%zu", i);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            ads111x_os_t os = ADS111X_OS_BUSY;
            while (os == ADS111X_OS_BUSY)
            {
                if (ads111x_read_os(&dev_handle, &os) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to read OS for AIN%zu", i);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (os == ADS111X_OS_READY)
            {
                int16_t res = 0;
                if (ads111x_read_conv(&dev_handle, &res) == ESP_OK)
                {
                    ESP_LOGI(TAG, "AIN%zu: raw=%d, volt=%.3f mV", i, res, res * lsb_size);
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to get conversion result for AIN%zu", i);
                }
            }
            else
            {
                ESP_LOGE(TAG, "Conversion did not complete for AIN%zu", i);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void vTaskMCP4725(void *pvParameter)
{
    i2c_master_dev_handle_t dev_handle;
    if (mcp4725_init(&dev_handle, I2C_PORT, MCP4725_ADDR_GND, I2C_SPEED_HZ) != ESP_OK ||
        mcp4725_write_register_data(&dev_handle, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MCP4725");
        vTaskDelete(NULL);
    }
    uint16_t val = 0;
    for (;;)
    {
        if (mcp4725_write_register_data(&dev_handle, val) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write register data");
        }
        else
        {
            ESP_LOGI(TAG, "DAC value: %d", val);
        }
        val += 20;
        if (val > 4095)
        {
            val = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void vTaskPCF8574A(void *pvParameter)
{
    pcf8574_device_t device;
    if (pcf8574_init(&device, I2C_PORT, PCF8574_ADDR_HHH, I2C_SPEED_HZ) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize PCF8574");
        vTaskDelete(NULL);
    }
    const pcf8574_pin_t outputs[] = {
        PCF8574_PIN_P0,
        PCF8574_PIN_P1,
        PCF8574_PIN_P2,
        PCF8574_PIN_P4,
        PCF8574_PIN_P5,
        PCF8574_PIN_P6,
        PCF8574_PIN_P7};
    const size_t num_outputs = sizeof(outputs) / sizeof(outputs[0]);
    for (;;)
    {
        for (size_t i = 0; i < num_outputs; i++)
        {
            if (pcf8574_toggle_pin(&device, outputs[i]) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to toggle P%d", outputs[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void vTaskPCF8574B(void *pvParameter)
{
    pcf8574_device_t device;
    if (pcf8574_init(&device, I2C_PORT, PCF8574_ADDR_HHL, I2C_SPEED_HZ) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize PCF8574");
        vTaskDelete(NULL);
    }
    const pcf8574_pin_t inputs[] = {
        PCF8574_PIN_P4,
        PCF8574_PIN_P5,
        PCF8574_PIN_P6,
        PCF8574_PIN_P7};
    const size_t num_inputs = sizeof(inputs) / sizeof(inputs[0]);
    const pcf8574_pin_t outputs[] = {
        PCF8574_PIN_P0,
        PCF8574_PIN_P1,
        PCF8574_PIN_P2};
    const size_t num_outputs = sizeof(outputs) / sizeof(outputs[0]);
    pcf8574_level_t level = PCF8574_LOW;
    for (;;)
    {
        for (size_t i = 0; i < num_outputs; i++)
        {
            if (pcf8574_toggle_pin(&device, outputs[i]) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to toggle P%d", outputs[i]);
            }
        }
        for (size_t i = 0; i < num_inputs; i++)
        {
            if (
                pcf8574_read_pin(&device, inputs[i], &level) == ESP_OK)
            {
                ESP_LOGI(TAG, "P%d: %s", inputs[i], level == PCF8574_LOW ? "LOW" : "HIGH");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read P%d", inputs[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}