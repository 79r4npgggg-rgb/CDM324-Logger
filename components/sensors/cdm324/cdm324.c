#include "cdm324.h"

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "board.h"
#include "cdm324_types.h"

static const char *TAG = "cdm324";

static QueueHandle_t s_event_queue = NULL;
static TaskHandle_t s_capture_task = NULL;
static volatile uint32_t s_last_edge_us = 0;
static volatile uint32_t s_last_time_us = 0;
static volatile int32_t s_last_level = 0;
static volatile int32_t s_last_freq_hz = 0;
static volatile int32_t s_last_velocity_mmps = 0;
static volatile uint32_t s_last_status = CDM324_STATUS_NORMAL;
static volatile uint32_t s_overflow_count = 0;
static volatile uint32_t s_dropped_samples = 0;

static void cdm324_capture_task(void *arg)
{
    (void)arg;
    cdm324_edge_event_t event;
    const app_config_t *config = app_config_get();

    while (1) {
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            uint32_t diff_us = 0;
            if (s_last_edge_us != 0 && event.timestamp_us >= s_last_edge_us) {
                diff_us = event.timestamp_us - s_last_edge_us;
            }

            if (diff_us == 0 || diff_us > 1000000U) {
                s_last_status = CDM324_STATUS_INVALID;
                s_last_level = 0;
                s_last_freq_hz = 0;
                s_last_velocity_mmps = 0;
                s_overflow_count++;
            } else {
                uint32_t freq_hz = 1000000U / diff_us;
                s_last_freq_hz = (int32_t)freq_hz;
                s_last_velocity_mmps = cdm324_velocity_mmps_from_freq_hz(s_last_freq_hz, config->wheel_diameter_mm);
                s_last_level = (s_last_freq_hz > 0) ? 1 : 0;
                s_last_status = CDM324_STATUS_NORMAL;
            }

            s_last_edge_us = event.timestamp_us;
            s_last_time_us = event.timestamp_us;
            vTaskDelay(pdMS_TO_TICKS(config->snapshot_period_ms));
        }
    }
}

bool cdm324_init(void)
{
    const app_config_t *config = app_config_get();
    s_event_queue = xQueueCreate(config->cdm324_queue_len, sizeof(cdm324_edge_event_t));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CDM324 event queue");
        return false;
    }

    cdm324_set_event_queue(s_event_queue);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_CDM324_INPUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO");
        return false;
    }

    if (gpio_install_isr_service(0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return false;
    }

    if (gpio_isr_handler_add(BOARD_GPIO_CDM324_INPUT, cdm324_gpio_isr_handler, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler");
        return false;
    }

    ESP_LOGI(TAG, "CDM324 initialized");
    return true;
}

bool cdm324_start(void)
{
    if (s_capture_task != NULL) {
        return true;
    }

    BaseType_t ret = xTaskCreate(cdm324_capture_task, "cdm324_capture", 4096, NULL, 5, &s_capture_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CDM324 capture task");
        return false;
    }

    return true;
}

bool cdm324_stop(void)
{
    if (s_capture_task == NULL) {
        return true;
    }

    vTaskDelete(s_capture_task);
    s_capture_task = NULL;
    return true;
}

bool cdm324_get_latest_snapshot(cdm324_snapshot_t *out_snapshot)
{
    if (out_snapshot == NULL) {
        return false;
    }

    out_snapshot->time_us = s_last_time_us;
    out_snapshot->status = s_last_status;
    out_snapshot->level = s_last_level;
    out_snapshot->freq_hz = s_last_freq_hz;
    out_snapshot->velocity_mmps = s_last_velocity_mmps;
    return true;
}

QueueHandle_t cdm324_get_event_queue(void)
{
    return s_event_queue;
}
