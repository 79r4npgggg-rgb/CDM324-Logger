#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

#define BOARD_GPIO_CDM324_INPUT    GPIO_NUM_4
#define BOARD_GPIO_LOG_BUTTON      GPIO_NUM_2  // Seeed XIAO Expansion Board D1 button -> GPIO2
/* Per the official Seeed XIAO pinout and SD sample code, the board LED is on GPIO21.
 * The SD card CS is D2 on the XIAO board, which maps to GPIO3. */
#define BOARD_GPIO_LED             GPIO_NUM_21
/* Seeed XIAO ESP32-S3 + Expansion Board V1.2:
 * OLED SSD1306 128x64, I2C address 0x3C, SDA=GPIO5 (D4), SCL=GPIO6 (D5). */
#define BOARD_I2C_SDA_GPIO         GPIO_NUM_5
#define BOARD_I2C_SCL_GPIO         GPIO_NUM_6
#define BOARD_OLED_I2C_ADDR        0x3C
#define BOARD_SD_SPI_HOST          SPI2_HOST
/* Official board pinout / Arduino sample code use D2 as the SD CS pin.
 * On the XIAO ESP32-S3 that is GPIO3, while SPI uses GPIO7/8/9. */
#define BOARD_SD_SPI_MOSI          GPIO_NUM_9
#define BOARD_SD_SPI_MISO          GPIO_NUM_8
#define BOARD_SD_SPI_SCLK          GPIO_NUM_7
#define BOARD_SD_SPI_CS            GPIO_NUM_3

void board_init(void);
void board_oled_init(void);
bool board_oled_probe(void);
void board_oled_clear(void);
void board_oled_write_text(const char *text);
void board_oled_show_status(int32_t raw_hz, int32_t velocity_mmps, bool logging_active);
void board_i2c_scan(void);
void board_diagnostics_run(void);
