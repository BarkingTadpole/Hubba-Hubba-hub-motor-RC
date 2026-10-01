#include <stdio.h>

#include "cli.h"
#include "dragy_sensor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "powertrain_controller.h"
#include "sdkconfig.h"
#include "telemetry_log.h"
#include "wifi_control.h"

static const char *TAG = "rc_car";

void app_main(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "Boot started (reset reason %d); IMU calibration runs once during this boot",
             (int)esp_reset_reason());

    esp_err_t err = powertrain_controller_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Controller initialization failed; drive remains unavailable: %s",
                 esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#if CONFIG_RC_DRAGY_LITE_ENABLED
    err = dragy_sensor_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Dragy Lite integration unavailable: %s", esp_err_to_name(err));
    }
#endif
    err = powertrain_controller_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Powertrain task failed to start; safe outputs remain active: %s",
                 esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    err = cli_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Serial CLI unavailable: %s", esp_err_to_name(err));
    }
    err = telemetry_log_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Offline CSV telemetry logging unavailable: %s",
                 esp_err_to_name(err));
    }
    err = wifi_control_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi monitoring/configuration unavailable: %s",
                 esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "RC car powertrain controller started");
    ESP_LOGI(TAG, "USB serial CLI baud: 115200");
#if CONFIG_RC_DRAGY_LITE_ENABLED
    ESP_LOGI(TAG, "UART1 routed to GPIO4/GPIO5 for Dragy Lite at %d baud",
             CONFIG_RC_DRAGY_UART_BAUD);
#endif
    ESP_LOGI(TAG, "Safe ESC outputs are active; torque vectoring remains runtime-gated");
}
