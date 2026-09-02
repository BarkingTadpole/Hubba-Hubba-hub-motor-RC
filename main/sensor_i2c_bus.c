#include "sensor_i2c_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "pin_config.h"

static i2c_master_bus_handle_t bus_handle;
static SemaphoreHandle_t bus_mutex;

esp_err_t sensor_i2c_bus_init(void)
{
    if (bus_handle != NULL && bus_mutex != NULL) {
        return ESP_OK;
    }

    if (bus_mutex == NULL) {
        bus_mutex = xSemaphoreCreateMutex();
        if (bus_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_IMU_I2C_SDA,
        .scl_io_num = PIN_IMU_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    return i2c_new_master_bus(&bus_config, &bus_handle);
}

esp_err_t sensor_i2c_bus_add_device(const i2c_device_config_t *config,
                                    i2c_master_dev_handle_t *device)
{
    if (config == NULL || device == NULL || bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sensor_i2c_bus_lock(20)) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = i2c_master_bus_add_device(bus_handle, config, device);
    sensor_i2c_bus_unlock();
    return err;
}

esp_err_t sensor_i2c_bus_probe(uint8_t address, uint32_t timeout_ms)
{
    if (bus_handle == NULL || address < 0x08 || address > 0x77) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor_i2c_bus_lock(timeout_ms + 5)) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = i2c_master_probe(bus_handle, address, (int)timeout_ms);
    sensor_i2c_bus_unlock();
    return err;
}

bool sensor_i2c_bus_lock(uint32_t timeout_ms)
{
    return bus_mutex != NULL &&
           xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void sensor_i2c_bus_unlock(void)
{
    if (bus_mutex != NULL) {
        xSemaphoreGive(bus_mutex);
    }
}
