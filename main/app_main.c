#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "board.h"
#include "cdm324.h"
#include "csv_logger.h"
#include "sdcard.h"

static const char *TAG = "app_main";

#define APP_BUTTON_POLL_MS         10U
#define APP_BUTTON_LONG_PRESS_MS   2000U
#define APP_LOG_TICK_MS            10U
#define APP_BUTTON_EVENT_QUEUE_LEN 4U

static bool s_logging_enabled = false;

static uint32_t s_last_logged_snapshot_time_us = 0U;

static uint32_t s_last_oled_update_ms = 0U;


typedef enum {
    APP_BUTTON_RELEASED,
    APP_BUTTON_PRESSED,
    APP_BUTTON_LONG_PRESS_DETECTED,
} app_button_state_t;


typedef enum {
    APP_BUTTON_EVENT_LONG_PRESS,
} app_button_event_t;


static QueueHandle_t s_button_event_queue = NULL;


/*
 * ============================================================
 * OLED
 * ============================================================
 */

static void app_show_oled_status(
    const cdm324_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }


    const uint32_t now_ms =
        (uint32_t)(
            esp_timer_get_time() /
            1000ULL);


    if (
        (now_ms -
         s_last_oled_update_ms) <
        200U
    ) {
        return;
    }


    s_last_oled_update_ms =
        now_ms;


    if (board_oled_probe()) {

        /*
         * The old OLED status function is designed
         * for FOUT frequency and velocity.
         *
         * For this first Aout version, show the
         * Doppler frequency and Aout DC level.
         */
        char line[32];

        snprintf(
            line,
            sizeof(line),
            "F:%ld DC:%ld",
            (long)snapshot->doppler_hz,
            (long)snapshot->aout_dc_mv);


        board_oled_write_text(line);
    }
}


/*
 * ============================================================
 * Logging control
 * ============================================================
 */

static void app_toggle_logging(void)
{
    s_logging_enabled =
        !s_logging_enabled;


    if (!csv_logger_set_logging_enabled(
            s_logging_enabled)) {

        ESP_LOGE(
            TAG,
            "Failed to %s logging",
            s_logging_enabled
                ? "start"
                : "stop");


        s_logging_enabled =
            false;


        return;
    }


    ESP_LOGI(
        TAG,
        "Logging %s",
        s_logging_enabled
            ? "enabled"
            : "disabled");

    if (s_logging_enabled) {
       s_last_logged_snapshot_time_us = 0U;
    }

    if (board_oled_probe()) {

        board_oled_write_text(
            s_logging_enabled
                ? "Now Logging..."
                : "Logging stopped");
    }
}


/*
 * ============================================================
 * Button
 * ============================================================
 */

static bool app_button_pressed(void)
{
    return
        gpio_get_level(
            BOARD_GPIO_LOG_BUTTON) == 0;
}


static void app_handle_button(void)
{
    static app_button_state_t state =
        APP_BUTTON_RELEASED;

    static uint32_t press_start_ms = 0U;


    const bool pressed =
        app_button_pressed();


    const uint32_t now_ms =
        (uint32_t)(
            esp_timer_get_time() /
            1000ULL);


    switch (state) {

    case APP_BUTTON_RELEASED:

        if (pressed) {

            press_start_ms =
                now_ms;

            state =
                APP_BUTTON_PRESSED;

            ESP_LOGI(
                TAG,
                "BUTTON PRESSED");
        }

        break;


    case APP_BUTTON_PRESSED:

        if (!pressed) {

            state =
                APP_BUTTON_RELEASED;

            ESP_LOGI(
                TAG,
                "BUTTON RELEASED");

        } else if (
            (now_ms -
             press_start_ms) >=
            APP_BUTTON_LONG_PRESS_MS
        ) {

            app_button_event_t event =
                APP_BUTTON_EVENT_LONG_PRESS;


            if (
                s_button_event_queue == NULL ||
                xQueueSend(
                    s_button_event_queue,
                    &event,
                    0) != pdTRUE
            ) {

                ESP_LOGW(
                    TAG,
                    "Button event queue full");
            }


            state =
                APP_BUTTON_LONG_PRESS_DETECTED;
        }

        break;


    case APP_BUTTON_LONG_PRESS_DETECTED:

        if (!pressed) {

            state =
                APP_BUTTON_RELEASED;

            ESP_LOGI(
                TAG,
                "BUTTON RELEASED");
        }

        break;
    }
}


static void app_button_task(void *arg)
{
    (void)arg;


    while (1) {

        app_handle_button();

        vTaskDelay(
            pdMS_TO_TICKS(
                APP_BUTTON_POLL_MS));
    }
}


/*
 * ============================================================
 * Main
 * ============================================================
 */

void app_main(void)
{
    app_config_init();

    board_init();


    gpio_set_pull_mode(
        BOARD_GPIO_LOG_BUTTON,
        GPIO_PULLUP_ONLY);

    gpio_set_direction(
        BOARD_GPIO_LOG_BUTTON,
        GPIO_MODE_INPUT);


    if (!sdcard_init()) {

        ESP_LOGE(
            TAG,
            "SD card init failed; logger will remain disabled");
    }


    if (!csv_logger_init()) {

        ESP_LOGE(
            TAG,
            "CSV logger init failed");
    }


    if (!cdm324_init()) {

        ESP_LOGE(
            TAG,
            "CDM324 Aout init failed");

        while (1) {

            vTaskDelay(
                pdMS_TO_TICKS(1000));
        }
    }


    if (!cdm324_start()) {

        ESP_LOGE(
            TAG,
            "CDM324 Aout start failed");

        while (1) {

            vTaskDelay(
                pdMS_TO_TICKS(1000));
        }
    }


    board_oled_init();


    if (board_oled_probe()) {

        board_oled_write_text(
            "CDM324 Aout Logger");
    }


    s_button_event_queue =
        xQueueCreate(
            APP_BUTTON_EVENT_QUEUE_LEN,
            sizeof(app_button_event_t));


    if (s_button_event_queue == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create button event queue");
    }


    BaseType_t button_task_result =
        xTaskCreate(
            app_button_task,
            "button",
            2048,
            NULL,
            5,
            NULL);


    if (button_task_result != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create button task");
    }


    ESP_LOGI(
        TAG,
        "Aout Logger ready. Hold button for 2s to toggle logging.");


    const app_config_t *config =
        app_config_get();


    uint32_t next_log_time_us =
        (uint32_t)esp_timer_get_time();


    while (1) {

        /*
         * ----------------------------------------------------
         * Button event
         * ----------------------------------------------------
         */

        app_button_event_t button_event;


        if (
            s_button_event_queue != NULL &&
            xQueueReceive(
                s_button_event_queue,
                &button_event,
                0) == pdTRUE
        ) {

            if (
                button_event ==
                APP_BUTTON_EVENT_LONG_PRESS
            ) {

                app_toggle_logging();
            }
        }


        /*
         * ----------------------------------------------------
         * Periodic logger snapshot
         * ----------------------------------------------------
         *
         * The actual ADC sampling and FFT run independently
         * in the CDM324 analysis task.
         *
         * This 10 ms loop only takes the latest result and
         * passes it to the SD writer queue.
         */

        const uint32_t now_us =
            (uint32_t)esp_timer_get_time();


        if (
            (int32_t)(
                now_us -
                next_log_time_us) >= 0
        ) {

            next_log_time_us +=
                config->snapshot_period_ms *
                1000U;


            cdm324_snapshot_t snapshot;


            if (
                cdm324_get_latest_snapshot(
                    &snapshot)
            ) {

                app_show_oled_status(
                    &snapshot);


if (csv_logger_is_logging_enabled()) {
    if (snapshot.time_us != s_last_logged_snapshot_time_us) {
    csv_snapshot_t csv_snapshot = {
        .time_us = snapshot.time_us,
        .aout_dc_mv = snapshot.aout_dc_mv,
        .aout_rms_mv = snapshot.aout_rms_mv,
        .aout_pp_mv = snapshot.aout_pp_mv,
        .doppler_hz = snapshot.doppler_hz,
        .status = snapshot.status,
        .peak_count = snapshot.peak_count,
    };

    for (uint32_t i = 0;
         i < CSV_MAX_PEAKS;
        ++i) {

        csv_snapshot.peaks[i].frequency_hz =
            snapshot.peaks[i].frequency_hz;

        csv_snapshot.peaks[i].power =
            snapshot.peaks[i].power;
    }

if (csv_logger_queue_snapshot(&csv_snapshot)) {
    s_last_logged_snapshot_time_us =
        snapshot.time_us;
}

        if (csv_logger_queue_snapshot(&csv_snapshot)) {
            s_last_logged_snapshot_time_us = snapshot.time_us;
        }
    }
}
            }
        }


        vTaskDelay(
            pdMS_TO_TICKS(
                APP_LOG_TICK_MS));
    }
}