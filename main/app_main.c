#include <stdio.h>

#include "cli.h"
#include "esp_err.h"
#include "esp_log.h"
#include "powertrain_controller.h"

static const char *TAG = "rc_car";

void app_main(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_ERROR_CHECK(powertrain_controller_init());
    powertrain_controller_start();
    cli_start();

    ESP_LOGI(TAG, "RC car powertrain controller started");
    ESP_LOGI(TAG, "USB serial CLI baud: 115200");
    ESP_LOGI(TAG, "Safe ESC outputs are active; torque vectoring defaults to disabled");
}
