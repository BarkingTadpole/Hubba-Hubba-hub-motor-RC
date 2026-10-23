idf_component_register(
    SRCS
        "app_main.c"
        "cli.c"
        "config_store.c"
        "cornering_control.c"
        "drivetrain_control.c"
        "esc_output.c"
        "powertrain_controller.c"
        "powertrain_commands.c"
        "powertrain_calibration.c"
        "powertrain_settings.c"
        "powertrain_diagnostics.c"
        "powertrain_steering.c"
        "rc_input.c"
        "remote_command.c"
        "rpm_sensor.c"
        "servo_output.c"
        "independent_steering.c"
        "steering_input_filter.c"
        "wifi_control.c"
    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_pcnt
        esp_timer
        esp_event
        esp_http_server
        esp_netif
        esp_wifi
        lwip
        nvs_flash
    INCLUDE_DIRS
        "."
        "include"
)

target_add_binary_data(${COMPONENT_LIB}
    "web/index.html" TEXT RENAME_TO "web_index")
target_add_binary_data(${COMPONENT_LIB}
    "web/app.js" TEXT RENAME_TO "web_app")
target_add_binary_data(${COMPONENT_LIB}
    "web/style.css" TEXT RENAME_TO "web_style")
