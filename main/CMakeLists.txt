idf_component_register(SRCS "rc_car_main.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_timer nvs_flash
                       INCLUDE_DIRS "")
