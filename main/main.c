#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ads111x.h"

// ================================ Public Defines ================================================

#define SDA_GPIO GPIO_NUM_21 // GPIO pin for SDA
#define SCL_GPIO GPIO_NUM_22 // GPIO pin for SCL
#define I2C_PORT I2C_NUM_0   // I2C port number

// ================================ Public Constants ==============================================

static const char *TAG = "MAIN";

// ================================ Public Functions Declaration ==================================

static void ads111x_test_task(void *pvParameter);

// ================================ Program Entry Point ===========================================

void app_main(void)
{
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(
        &ads111x_test_task,
        "ads111x_test_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        5,
        NULL,
        APP_CPU_NUM);
    if (xReturned != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ads111x_task");
    }
    vTaskDelete(NULL);
}

// ================================ Public Functions Definitions ==================================

static void ads111x_test_task(void *pvParameter)
{
    ads111x_dev_t dev;
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

    ads111x_mux_t mux;
    if (ads111x_get_mux(&dev, &mux) == ESP_OK)
    {
        ESP_LOGI(TAG, "Input multiplexer: %d", mux);
    }

    ads111x_pga_t pga;
    if (ads111x_get_pga(&dev, &pga) == ESP_OK)
    {
        ESP_LOGI(TAG, "Programmable gain amplifier: %d", pga);
    }

    ads111x_mode_t mode;
    if (ads111x_get_mode(&dev, &mode) == ESP_OK)
    {
        ESP_LOGI(TAG, "Operating mode: %d", mode);
    }

    ads111x_dr_t dr;
    if (ads111x_get_dr(&dev, &dr) == ESP_OK)
    {
        ESP_LOGI(TAG, "Data rate: %d", dr);
    }

    ads111x_comp_mode_t comp;
    if (ads111x_get_comp_mode(&dev, &comp) == ESP_OK)
    {
        ESP_LOGI(TAG, "Comparator mode: %d", comp);
    }

    ads111x_comp_pol_t pol;
    if (ads111x_get_comp_pol(&dev, &pol) == ESP_OK)
    {
        ESP_LOGI(TAG, "Comparator polarity: %d", pol);
    }

    ads111x_comp_lat_t lat;
    if (ads111x_get_comp_lat(&dev, &lat) == ESP_OK)
    {
        ESP_LOGI(TAG, "Comparator latch: %d", lat);
    }

    ads111x_comp_que_t que;
    if (ads111x_get_comp_que(&dev, &que) == ESP_OK)
    {
        ESP_LOGI(TAG, "Comparator queue: %d", que);
    }

    int16_t lo_thresh;
    if (ads111x_get_comp_lo_thresh(&dev, &lo_thresh) == ESP_OK)
    {
        ESP_LOGI(TAG, "Comparator low threshold: %d", lo_thresh);
    }

    int16_t hi_thresh;
    if (ads111x_get_comp_hi_thresh(&dev, &hi_thresh) == ESP_OK)
    {
        ESP_LOGI(TAG, "Comparator high threshold: %d", hi_thresh);
    }

    int16_t val;
    if (ads111x_get_conv(&dev, &val) == ESP_OK)
    {
        ESP_LOGI(TAG, "Conversion value: %d", val);
    }

    vTaskDelete(NULL);
}