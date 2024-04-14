#include <string.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <bmp280.h>
#include <tsl2561.h>
#include "enviro_sensor.h"

#define SENSOR_I2C_MASTER_SDA   CONFIG_SENSOR_I2C_MASTER_SDA
#define SENSOR_I2C_MASTER_SCL   CONFIG_SENSOR_I2C_MASTER_SCL

#define TAG "enviro_sensor"

static enviro_ready_f enviro_cb;
static bmp280_t dev_BMP280;
static tsl2561_t dev_TSL2561;

static const char* INTEGRATION_TIME_STR[3] = {
    "13",
    "101",
    "402",
};

static const char* INTR_CONTROL_STR[4] = {
    "Disabled",
    "Level",
    "SMBAlert",
    "Test mode",
};

static const char* INTR_PERSISTENCE_STR[16] = {
    "Every cycle",
    "Any value out of range",
    "2 integration periods out of range",
    "3 integration periods out of range",
    "4 integration periods out of range",
    "5 integration periods out of range",
    "6 integration periods out of range",
    "7 integration periods out of range",
    "8 integration periods out of range",
    "9 integration periods out of range",
    "10 integration periods out of range",
    "11 integration periods out of range",
    "12 integration periods out of range",
    "13 integration periods out of range",
    "14 integration periods out of range",
    "15 integration periods out of range",
};

static void enviro_sensor_read(void *arg)
{
    int mS = (int) arg;
    float pressure, temperature, humidity;
    uint32_t lux;
    esp_err_t resBMP280, resTSL2561;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(mS));
        resBMP280 = bmp280_read_float(&dev_BMP280, &temperature, &pressure, &humidity); // gather BMP280 data
        resTSL2561 = tsl2561_read_lux(&dev_TSL2561, &lux);      // gather TSL2561

        if(enviro_cb && resBMP280 == ESP_OK && resTSL2561 == ESP_OK){       // all sensors read
            pressure /= 100;
            ESP_LOGD(TAG, "Pressure: %.2f kPa, Temperature: %.2f C, Humidity: %.2f, Lux: %lu", pressure, temperature, humidity, lux);
            enviro_cb(pressure, temperature, humidity, lux);
        }else if (enviro_cb && resBMP280 == ESP_OK){            // only BMP280 read
            pressure /= 100;
            ESP_LOGD(TAG, "No Lux\tPressure: %.2f kPa, Temperature: %.2f C, Humidity: %.2f", pressure, temperature, humidity);
            enviro_cb(pressure, temperature, humidity, 0);
        }else if (enviro_cb && resTSL2561 == ESP_OK){           // only TSL2561 read
            ESP_LOGD(TAG, "No BMP280\tLux: %lu", lux);
            enviro_cb(0, 0, 0, lux);
        }else{                              // none read
            ESP_LOGD(TAG, "No sensor readings");
        }
    }
    // if BMP280 not reading, make all params zero
    // if TSL2561 not reading, make lux param zero
    // push all readings if both sensors work
}

esp_err_t enviro_sensor_init(enviro_ready_f cb, int update_interval)
{
    enviro_cb = cb;

    bmp280_params_t params;
    bmp280_init_default_params(&params);
    // bmp280_t dev_BMP280;
    memset(&dev_BMP280, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(bmp280_init_desc(&dev_BMP280, BMP280_I2C_ADDRESS_0, 0, SENSOR_I2C_MASTER_SDA, SENSOR_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev_BMP280, &params));

    bool bme280p = dev_BMP280.id == BME280_CHIP_ID;
    ESP_LOGI(TAG, "BMP280: found %s", bme280p ? "BME280" : "BMP280");

    // initialize TSL2561 with default values
    ESP_ERROR_CHECK(tsl2561_init_desc(&dev_TSL2561, TSL2561_I2C_ADDR_FLOAT, 0, SENSOR_I2C_MASTER_SDA, SENSOR_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(tsl2561_init(&dev_TSL2561));

    ESP_LOGI(TAG, "Found TSL2561 in package %s", dev_TSL2561.package_type == TSL2561_PACKAGE_CS ? "CS" : "T/FN/CL");
    ESP_LOGI(TAG, "Gain: %s", dev_TSL2561.gain == TSL2561_GAIN_1X ? "1X" : "16X");
    ESP_LOGI(TAG, "Integration time: %s ms", INTEGRATION_TIME_STR[dev_TSL2561.integration_time]);
    ESP_LOGI(TAG, "Interrupt control: %s", INTR_CONTROL_STR[dev_TSL2561.interrupt_control >> 4]);
    if (dev_TSL2561.interrupt_control != 0) {
        ESP_LOGI(TAG, "Persistence: %s", INTR_PERSISTENCE_STR[dev_TSL2561.interrupt_persistence]);
        ESP_LOGI(TAG, "Low Threshold: %x, High Threshold: %x", dev_TSL2561.low_threshold, dev_TSL2561.high_threshold);
    }

    xTaskCreate(enviro_sensor_read, TAG, 8 * configMINIMAL_STACK_SIZE, (void *) update_interval, 5, NULL);

    return ESP_OK;
}