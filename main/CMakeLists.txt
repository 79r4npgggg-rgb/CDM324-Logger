idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS ""
                       REQUIRES board config storage cdm324
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_spi esp_timer)
