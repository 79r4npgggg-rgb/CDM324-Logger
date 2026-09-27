idf_component_register(
    SRCS "input_diag.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio board
)