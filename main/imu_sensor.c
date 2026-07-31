#include "imu_sensor.h"

#include <math.h>
#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pin_config.h"

#define ISM330DHCX_ADDRESS 0x6A
#define ISM330DHCX_WHO_AM_I 0x0F
#define ISM330DHCX_WHO_AM_I_VALUE 0x6B
#define ISM330DHCX_CTRL1_XL 0x10
#define ISM330DHCX_CTRL2_G 0x11
#define ISM330DHCX_CTRL3_C 0x12
#define ISM330DHCX_OUTX_L_G 0x22
#define IMU_STALE_TIMEOUT_US 100000

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t imu_device;
static SemaphoreHandle_t i2c_mutex;
static imu_snapshot_t current_snapshot;
static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static float gyro_z_bias_dps;
static int8_t configured_yaw_sign = 1;
static bool initialized;
static bool bias_calibrated;

static int16_t read_i16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command[2] = {reg, value};
    return i2c_master_transmit(imu_device, command, sizeof(command), 50);
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t data_size)
{
    return i2c_master_transmit_receive(imu_device, &reg, 1, data, data_size, 50);
}

static bool read_motion(float gyro_dps[3], float accel_mps2[3])
{
    uint8_t raw[12];

    if (!initialized || xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }
    esp_err_t err = read_registers(ISM330DHCX_OUTX_L_G, raw, sizeof(raw));
    xSemaphoreGive(i2c_mutex);
    if (err != ESP_OK) {
        return false;
    }

    const float gyro_scale_dps = 0.035f;
    const float accel_scale_mps2 = 0.00478528f;
    for (size_t axis = 0; axis < 3; axis++) {
        gyro_dps[axis] = (float)read_i16(&raw[axis * 2]) * gyro_scale_dps;
        accel_mps2[axis] = (float)read_i16(&raw[6 + axis * 2]) * accel_scale_mps2;
    }
    return true;
}

esp_err_t imu_sensor_init(int8_t yaw_sign)
{
    if (yaw_sign != -1 && yaw_sign != 1) {
        return ESP_ERR_INVALID_ARG;
    }
    configured_yaw_sign = yaw_sign;

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_IMU_I2C_SDA,
        .scl_io_num = PIN_IMU_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ISM330DHCX_ADDRESS,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(i2c_bus, &device_config, &imu_device);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t who_am_i = 0;
    err = read_registers(ISM330DHCX_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (who_am_i != ISM330DHCX_WHO_AM_I_VALUE) {
        return ESP_ERR_NOT_FOUND;
    }

    err = write_register(ISM330DHCX_CTRL3_C, 0x44);
    if (err == ESP_OK) {
        err = write_register(ISM330DHCX_CTRL1_XL, 0x54);
    }
    if (err == ESP_OK) {
        err = write_register(ISM330DHCX_CTRL2_G, 0x58);
    }
    if (err != ESP_OK) {
        return err;
    }

    initialized = true;
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

void imu_sensor_set_yaw_sign(int8_t yaw_sign)
{
    if (yaw_sign == -1 || yaw_sign == 1) {
        configured_yaw_sign = yaw_sign;
    }
}

bool imu_sensor_update(void)
{
    float gyro_dps[3];
    float accel_mps2[3];
    int64_t now_us = esp_timer_get_time();

    if (!read_motion(gyro_dps, accel_mps2)) {
        portENTER_CRITICAL(&snapshot_lock);
        if ((now_us - current_snapshot.updated_at_us) > IMU_STALE_TIMEOUT_US) {
            current_snapshot.valid = false;
        }
        portEXIT_CRITICAL(&snapshot_lock);
        return false;
    }

    gyro_dps[2] -= gyro_z_bias_dps;

    portENTER_CRITICAL(&snapshot_lock);
    for (size_t axis = 0; axis < 3; axis++) {
        current_snapshot.gyro_dps[axis] = gyro_dps[axis];
        current_snapshot.accel_mps2[axis] = accel_mps2[axis];
    }
    current_snapshot.yaw_rate_dps = gyro_dps[2] * (float)configured_yaw_sign;
    current_snapshot.updated_at_us = now_us;
    current_snapshot.valid = true;
    current_snapshot.bias_calibrated = bias_calibrated;
    portEXIT_CRITICAL(&snapshot_lock);
    return true;
}

bool imu_sensor_calibrate_bias(uint32_t duration_ms)
{
    if (!initialized || duration_ms < 500) {
        return false;
    }

    double sum = 0.0;
    double sum_squared = 0.0;
    double accel_magnitude_sum = 0.0;
    uint32_t samples = 0;
    int64_t end_at_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);

    while (esp_timer_get_time() < end_at_us) {
        float gyro_dps[3];
        float accel_mps2[3];
        if (read_motion(gyro_dps, accel_mps2)) {
            double yaw = gyro_dps[2];
            sum += yaw;
            sum_squared += yaw * yaw;
            accel_magnitude_sum += sqrt((double)accel_mps2[0] * accel_mps2[0] +
                                        (double)accel_mps2[1] * accel_mps2[1] +
                                        (double)accel_mps2[2] * accel_mps2[2]);
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (samples < 100) {
        return false;
    }

    double mean = sum / samples;
    double variance = (sum_squared / samples) - (mean * mean);
    double accel_magnitude = accel_magnitude_sum / samples;
    if (variance > 9.0 || accel_magnitude < 7.0 || accel_magnitude > 12.5) {
        return false;
    }

    gyro_z_bias_dps = (float)mean;
    bias_calibrated = true;
    imu_sensor_update();
    return true;
}

void imu_sensor_get_snapshot(imu_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&snapshot_lock);
    *snapshot = current_snapshot;
    if ((esp_timer_get_time() - snapshot->updated_at_us) > IMU_STALE_TIMEOUT_US) {
        snapshot->valid = false;
    }
    portEXIT_CRITICAL(&snapshot_lock);
}
