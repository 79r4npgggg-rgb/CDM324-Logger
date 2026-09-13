idf_component_register(SRCS "cdm324.c" "cdm324_isr.c" "cdm324_velocity.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_gpio esp_timer freertos board config storage)
