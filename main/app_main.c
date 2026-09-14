#include <stdbool.h>
#include <stdint.h>
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

#define APP_BUTTON_POLL_MS       10U
#define APP_BUTTON_LONG_PRESS_MS 2000U
#define APP_LOG_TICK_MS          10U

static bool s_logging_enabled = false;
static uint32_t s_last_oled_update_ms = 0U;

static void app_show_oled_status(const cdm324_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((now_ms - s_last_oled_update_ms) < 1000U) {
        return;
    }
    s_last_oled_update_ms = now_ms;

    if (board_oled_probe()) {
        board_oled_show_status(snapshot->freq_hz, snapshot->velocity_mmps, s_logging_enabled);
    }
}

static void app_toggle_logging(void)
{
    s_logging_enabled = !s_logging_enabled;
    if (!csv_logger_set_logging_enabled(s_logging_enabled)) {
        ESP_LOGE(TAG, "Failed to %s logging", s_logging_enabled ? "start" : "stop");
        s_logging_enabled = false;
        return;
    }

    ESP_LOGI(TAG, "Logging %s", s_logging_enabled ? "enabled" : "disabled");
    if (board_oled_probe()) {
        board_oled_write_text(s_logging_enabled ? "Now Logging..." : "Logging stopped");
    }
}

static bool app_button_pressed(void)
{
    return gpio_get_level(BOARD_GPIO_LOG_BUTTON) == 0;
}

static void app_handle_button(void)
{
    static bool last_pressed = false;

    const bool pressed = app_button_pressed();

    ESP_LOGI(TAG, "GPIO2=%d", gpio_get_level(GPIO_NUM_2));

    if (pressed != last_pressed) {
        ESP_LOGI(TAG, "BUTTON %s", pressed ? "PRESSED" : "RELEASED");
        last_pressed = pressed;
    }
}

void app_main(void)
{
    app_config_init();
    board_init();

    gpio_set_pull_mode(BOARD_GPIO_LOG_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_direction(BOARD_GPIO_LOG_BUTTON, GPIO_MODE_INPUT);

    if (!sdcard_init()) {
        ESP_LOGE(TAG, "SD card init failed; logger will remain disabled");
    }

    if (!csv_logger_init()) {
        ESP_LOGE(TAG, "CSV logger init failed");
    }

    if (!cdm324_init()) {
        ESP_LOGE(TAG, "CDM324 init failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (!cdm324_start()) {
        ESP_LOGE(TAG, "CDM324 start failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    board_oled_init();
    if (board_oled_probe()) {
        board_oled_write_text("CDM324 Logger");
    }

    ESP_LOGI(TAG, "Logger ready. Hold button for 2s to toggle logging.");

    uint32_t last_snapshot_time_us = 0;
    while (1) {
        app_handle_button();

        cdm324_snapshot_t snapshot;
        if (cdm324_get_latest_snapshot(&snapshot)) {
            if (snapshot.time_us != last_snapshot_time_us) {
                last_snapshot_time_us = snapshot.time_us;
                app_show_oled_status(&snapshot);

                if (s_logging_enabled) {
                    csv_snapshot_t csv_snapshot = {
                        .time_us = snapshot.time_us,
                        .level = snapshot.level,
                        .freq_hz = snapshot.freq_hz,
                        .velocity_mmps = snapshot.velocity_mmps,
                        .status = snapshot.status,
                    };
                    csv_logger_queue_snapshot(&csv_snapshot);
                    csv_logger_flush_pending();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_LOG_TICK_MS));
    }
}
