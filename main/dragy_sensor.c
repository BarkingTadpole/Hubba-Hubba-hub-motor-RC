#include "dragy_sensor.h"

#include "sdkconfig.h"

#if CONFIG_RC_DRAGY_LITE_ENABLED

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "dragy_nmea.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "sensor_i2c_bus.h"

#define DRAGY_UART UART_NUM_1
#define DRAGY_UART_RX_BUFFER_BYTES 4096
#define DRAGY_TASK_STACK_BYTES 4096
#define DRAGY_TASK_PRIORITY 2
#define DRAGY_STALE_TIMEOUT_US 1500000
#define IMU_I2C_ADDRESS 0x6A

static const char *TAG = "dragy_sensor";
static dragy_sensor_snapshot_t current_snapshot;
static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t dragy_task_handle;
static uint8_t raw_capture_bytes[DRAGY_RAW_CAPTURE_BYTES];
static size_t raw_capture_write_index;
static size_t raw_capture_length;
static uint32_t raw_capture_total_received;
static uint32_t raw_capture_overwritten;
static StaticSemaphore_t raw_capture_mutex_storage;
static SemaphoreHandle_t raw_capture_mutex;

static void capture_raw_bytes(const uint8_t *bytes, size_t length)
{
    if (bytes == NULL || length == 0 || raw_capture_mutex == NULL) return;

    xSemaphoreTake(raw_capture_mutex, portMAX_DELAY);
    for (size_t index = 0; index < length; index++) {
        raw_capture_bytes[raw_capture_write_index] = bytes[index];
        raw_capture_write_index =
            (raw_capture_write_index + 1U) % DRAGY_RAW_CAPTURE_BYTES;
        if (raw_capture_length < DRAGY_RAW_CAPTURE_BYTES) {
            raw_capture_length++;
        } else {
            raw_capture_overwritten++;
        }
        raw_capture_total_received++;
    }
    xSemaphoreGive(raw_capture_mutex);
}

static void scan_shared_i2c_bus(void)
{
    dragy_sensor_snapshot_t scan = {0};
    for (uint8_t address = 0x08; address <= 0x77; address++) {
        if (sensor_i2c_bus_probe(address, 2) != ESP_OK) continue;
        if (scan.i2c_device_count < DRAGY_I2C_MAX_DEVICES) {
            scan.i2c_addresses[scan.i2c_device_count++] = address;
        }
        if (address != IMU_I2C_ADDRESS) {
            scan.compass_candidate_present = true;
        }
    }
    scan.i2c_scan_complete = true;

    portENTER_CRITICAL(&snapshot_lock);
    current_snapshot.i2c_device_count = scan.i2c_device_count;
    memcpy(current_snapshot.i2c_addresses, scan.i2c_addresses,
           sizeof(current_snapshot.i2c_addresses));
    current_snapshot.i2c_scan_complete = true;
    current_snapshot.compass_candidate_present = scan.compass_candidate_present;
    portEXIT_CRITICAL(&snapshot_lock);

    char addresses[DRAGY_I2C_MAX_DEVICES * 5] = {0};
    size_t used = 0;
    for (uint8_t index = 0; index < scan.i2c_device_count; index++) {
        int written = snprintf(addresses + used, sizeof(addresses) - used,
                               "%s0x%02X", index == 0 ? "" : ",",
                               scan.i2c_addresses[index]);
        if (written < 0 || (size_t)written >= sizeof(addresses) - used) break;
        used += (size_t)written;
    }
    ESP_LOGI(TAG, "Shared I2C scan found %u device(s): %s",
             scan.i2c_device_count,
             scan.i2c_device_count > 0 ? addresses : "none");
    if (scan.compass_candidate_present) {
        ESP_LOGI(TAG, "Non-IMU I2C response detected; report its address before selecting a compass driver");
    } else {
        ESP_LOGW(TAG, "No Dragy compass candidate detected; power it before ESP32 startup and check SDA/SCL wiring");
    }
}

static void update_snapshot(const dragy_nmea_parser_t *parser,
                            uint32_t received_bytes,
                            bool sentence_updated,
                            int64_t now_us)
{
    portENTER_CRITICAL(&snapshot_lock);
    current_snapshot.uart_bytes += received_bytes;
    current_snapshot.valid_sentence_count = parser->fix.valid_sentence_count;
    current_snapshot.checksum_error_count = parser->fix.checksum_error_count;
    current_snapshot.parse_error_count = parser->fix.parse_error_count;
    if (sentence_updated) {
        current_snapshot.latitude_deg = parser->fix.latitude_deg;
        current_snapshot.longitude_deg = parser->fix.longitude_deg;
        current_snapshot.speed_mps = parser->fix.speed_mps;
        current_snapshot.course_deg = parser->fix.course_deg;
        current_snapshot.altitude_m = parser->fix.altitude_m;
        current_snapshot.hdop = parser->fix.hdop;
        current_snapshot.utc_time_ms = parser->fix.utc_time_ms;
        current_snapshot.date_ddmmyy = parser->fix.date_ddmmyy;
        current_snapshot.satellites = parser->fix.satellites;
        current_snapshot.fix_valid = parser->fix.fix_valid;
        current_snapshot.position_valid = parser->fix.position_valid;
        current_snapshot.speed_valid = parser->fix.speed_valid;
        current_snapshot.course_valid = parser->fix.course_valid;
        current_snapshot.altitude_valid = parser->fix.altitude_valid;
        current_snapshot.updated_at_us = now_us;
        current_snapshot.nmea_recent = true;
    } else if (current_snapshot.updated_at_us > 0 &&
               now_us - current_snapshot.updated_at_us > DRAGY_STALE_TIMEOUT_US) {
        current_snapshot.nmea_recent = false;
        current_snapshot.fix_valid = false;
        current_snapshot.position_valid = false;
        current_snapshot.speed_valid = false;
        current_snapshot.course_valid = false;
        current_snapshot.altitude_valid = false;
    }
    portEXIT_CRITICAL(&snapshot_lock);
}

static void dragy_task(void *arg)
{
    (void)arg;
    dragy_nmea_parser_t parser;
    dragy_nmea_init(&parser);
    uint8_t bytes[256];

    while (true) {
        int received = uart_read_bytes(DRAGY_UART, bytes, sizeof(bytes),
                                       pdMS_TO_TICKS(25));
        bool updated = false;
        if (received > 0) {
            capture_raw_bytes(bytes, (size_t)received);
            for (int index = 0; index < received; index++) {
                if (dragy_nmea_feed(&parser, bytes[index])) updated = true;
            }
        }
        update_snapshot(&parser, received > 0 ? (uint32_t)received : 0,
                        updated, esp_timer_get_time());
    }
}

esp_err_t dragy_sensor_start(void)
{
    if (dragy_task_handle != NULL) return ESP_ERR_INVALID_STATE;

    if (raw_capture_mutex == NULL) {
        raw_capture_mutex =
            xSemaphoreCreateMutexStatic(&raw_capture_mutex_storage);
    }
    if (raw_capture_mutex == NULL) return ESP_ERR_NO_MEM;

    esp_err_t err = sensor_i2c_bus_init();
    if (err == ESP_OK) {
        scan_shared_i2c_bus();
    } else {
        ESP_LOGW(TAG, "Deferred compass discovery unavailable; GPS UART will continue: %s",
                 esp_err_to_name(err));
    }

    uart_config_t uart_config = {
        .baud_rate = CONFIG_RC_DRAGY_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(DRAGY_UART, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(DRAGY_UART, PIN_DRAGY_UART_TX, PIN_DRAGY_UART_RX,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        err = uart_driver_install(DRAGY_UART, DRAGY_UART_RX_BUFFER_BYTES,
                                  0, 0, NULL, 0);
    }
    if (err != ESP_OK) return err;
    uart_flush_input(DRAGY_UART);

    portENTER_CRITICAL(&snapshot_lock);
    current_snapshot.initialized = true;
    current_snapshot.baud_rate = CONFIG_RC_DRAGY_UART_BAUD;
    portEXIT_CRITICAL(&snapshot_lock);

    BaseType_t created = xTaskCreatePinnedToCore(dragy_task,
                                                 "dragy_gps",
                                                 DRAGY_TASK_STACK_BYTES,
                                                 NULL,
                                                 DRAGY_TASK_PRIORITY,
                                                 &dragy_task_handle,
                                                 0);
    if (created != pdPASS) {
        uart_driver_delete(DRAGY_UART);
        portENTER_CRITICAL(&snapshot_lock);
        current_snapshot.initialized = false;
        portEXIT_CRITICAL(&snapshot_lock);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void dragy_sensor_get_snapshot(dragy_sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    portENTER_CRITICAL(&snapshot_lock);
    *snapshot = current_snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
}

void dragy_sensor_reset_raw_capture(void)
{
    if (raw_capture_mutex == NULL) return;
    xSemaphoreTake(raw_capture_mutex, portMAX_DELAY);
    raw_capture_write_index = 0;
    raw_capture_length = 0;
    raw_capture_total_received = 0;
    raw_capture_overwritten = 0;
    xSemaphoreGive(raw_capture_mutex);
}

void dragy_sensor_get_raw_capture(dragy_raw_capture_t *capture)
{
    if (capture == NULL) return;
    memset(capture, 0, sizeof(*capture));
    if (raw_capture_mutex == NULL) return;

    xSemaphoreTake(raw_capture_mutex, portMAX_DELAY);
    capture->length = raw_capture_length;
    capture->total_received = raw_capture_total_received;
    capture->overwritten = raw_capture_overwritten;
    size_t oldest = (raw_capture_write_index + DRAGY_RAW_CAPTURE_BYTES -
                     raw_capture_length) % DRAGY_RAW_CAPTURE_BYTES;
    for (size_t index = 0; index < raw_capture_length; index++) {
        capture->bytes[index] =
            raw_capture_bytes[(oldest + index) % DRAGY_RAW_CAPTURE_BYTES];
    }
    xSemaphoreGive(raw_capture_mutex);
}

#else

#include <string.h>

esp_err_t dragy_sensor_start(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

void dragy_sensor_get_snapshot(dragy_sensor_snapshot_t *snapshot)
{
    if (snapshot != NULL) memset(snapshot, 0, sizeof(*snapshot));
}

void dragy_sensor_reset_raw_capture(void)
{
}

void dragy_sensor_get_raw_capture(dragy_raw_capture_t *capture)
{
    if (capture != NULL) memset(capture, 0, sizeof(*capture));
}

#endif
