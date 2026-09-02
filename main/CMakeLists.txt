idf_component_register(
    SRCS
        "app_main.c"
        "cli.c"
        "config_store.c"
        "cornering_control.c"
        "dragy_nmea.c"
        "dragy_sensor.c"
        "drivetrain_control.c"
        "esc_output.c"
        "imu_sensor.c"
        "powertrain_controller.c"
        "rc_input.c"
        "remote_command.c"
        "rpm_sensor.c"
        "sensor_i2c_bus.c"
        "servo_output.c"
        "steering_input_filter.c"
        "steering_curve.c"
        "telemetry_log.c"
        "torque_vectoring.c"
        "wifi_control.c"
    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_pcnt
        esp_driver_uart
        esp_timer
        esp_event
        esp_http_server
        esp_netif
        esp_wifi
        fatfs
        lwip
        nvs_flash
        wear_levelling
    INCLUDE_DIRS
        "."
        "include"
)

target_add_binary_data(${COMPONENT_LIB}
    "../tools/wifi_bridge/web/index.html" TEXT RENAME_TO "web_index")
target_add_binary_data(${COMPONENT_LIB}
    "../tools/wifi_bridge/web/app.js" TEXT RENAME_TO "web_app")
target_add_binary_data(${COMPONENT_LIB}
    "../tools/wifi_bridge/web/style.css" TEXT RENAME_TO "web_style")
