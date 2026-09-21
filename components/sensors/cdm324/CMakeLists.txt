idf_component_register(
    SRCS
        "cdm324.c"

    INCLUDE_DIRS
        "include"

    REQUIRES
        esp_driver_gpio
        esp_timer
        freertos
        esp_adc
        esp-dsp
        board
        config
        storage
        espressif__esp-dsp
)