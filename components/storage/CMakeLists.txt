idf_component_register(
    SRCS "sdcard.c"
         "csv_logger.c"
         "spectrum_logger.c"
    INCLUDE_DIRS "include"
    REQUIRES fatfs
             vfs
             board
             config
             cdm324
             esp_driver_sdmmc
             esp_driver_sdspi
             esp_timer
             esp_common
)