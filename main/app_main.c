#include <stdio.h>

#include "cli.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "powertrain_controller.h"

static const char *TAG = "rc_car";

void app_main(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    esp_err_t err = powertrain_controller_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Controller initialization failed; drive remains unavailable: %s",
                 esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
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

    ESP_LOGI(TAG, "RC car powertrain controller started");
    ESP_LOGI(TAG, "USB serial CLI baud: 115200");
    ESP_LOGI(TAG, "Safe ESC outputs are active; torque vectoring defaults to disabled");
}
