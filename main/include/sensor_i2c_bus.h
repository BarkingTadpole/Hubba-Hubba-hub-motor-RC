#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t sensor_i2c_bus_init(void);
esp_err_t sensor_i2c_bus_add_device(const i2c_device_config_t *config,
                                    i2c_master_dev_handle_t *device);
esp_err_t sensor_i2c_bus_probe(uint8_t address, uint32_t timeout_ms);
bool sensor_i2c_bus_lock(uint32_t timeout_ms);
void sensor_i2c_bus_unlock(void);
