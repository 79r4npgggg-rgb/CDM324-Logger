#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "board.h"
#include "cdm324.h"
#include "csv_logger.h"
#include "sdcard.h"

static const char *TAG = "app_main";

#define APP_BUTTON_LONG_PRESS_US 2000000UL
#define TEST_MODE_I2C_SCAN 0
#define TEST_MODE_SD 1
#define TEST_MODE_BOTH 2
#define TEST_MODE TEST_MODE_I2C_SCAN

void app_main(void)
{
    app_config_init();
    board_init();

#if TEST_MODE == TEST_MODE_I2C_SCAN
    ESP_LOGI(TAG, "=== I2C SCAN ONLY TEST START ===");
    ESP_LOGI(TAG, "Target: SDA=GPIO5, SCL=GPIO6, scan 0x01..0x7F");
    board_i2c_scan();
    ESP_LOGI(TAG, "I2C scan complete. No OLED draw or init attempted.");
    ESP_LOGI(TAG, "I2C-only test complete. Waiting for reset.");
#elif TEST_MODE == TEST_MODE_SD
    ESP_LOGI(TAG, "=== SD ONLY TEST START ===");
    const bool sd_ok = sdcard_probe();
    ESP_LOGI(TAG, "SD TEST RESULT: %s", sd_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "SD-only test complete. Waiting for reset.");
#else
    ESP_LOGI(TAG, "=== OLED TEST START ===");
    const bool oled_ok = board_oled_probe();
    ESP_LOGI(TAG, "OLED TEST RESULT: %s", oled_ok ? "PASS" : "FAIL");

    ESP_LOGI(TAG, "=== SD TEST START ===");
    const bool sd_ok = sdcard_probe();
    ESP_LOGI(TAG, "SD TEST RESULT: %s", sd_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "Hardware test complete. Waiting for reset.");
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
