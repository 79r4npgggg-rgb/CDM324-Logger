#include "csv_logger.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "app_config.h"
#include "sdcard.h"

static const char *TAG = "csv_logger";
static QueueHandle_t s_snapshot_queue = NULL;
static uint32_t s_overflow_count = 0;
static uint32_t s_dropped_samples = 0;
static bool s_logging_enabled = false;

static void csv_logger_format_line(char *out, size_t out_size, const csv_snapshot_t *snapshot)
{
    if (out == NULL || snapshot == NULL || out_size == 0) {
        return;
    }

    snprintf(out, out_size,
             "%lu,%ld,%ld,%ld,%lu\n",
             (unsigned long)snapshot->time_us,
             (long)snapshot->level,
             (long)snapshot->freq_hz,
             (long)snapshot->velocity_mmps,
             (unsigned long)snapshot->status);
}

bool csv_logger_init(void)
{
    const app_config_t *config = app_config_get();
    s_snapshot_queue = xQueueCreate(config->csv_buffer_lines, sizeof(csv_snapshot_t));
    if (s_snapshot_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CSV queue");
        return false;
    }

    s_logging_enabled = false;
    return true;
}

bool csv_logger_queue_snapshot(const csv_snapshot_t *snapshot)
{
    if (snapshot == NULL || s_snapshot_queue == NULL || !s_logging_enabled) {
        return false;
    }

    if (xQueueSend(s_snapshot_queue, snapshot, 0) != pdTRUE) {
        s_dropped_samples++;
        return false;
    }

    return true;
}

bool csv_logger_flush_pending(void)
{
    if (s_snapshot_queue == NULL || !s_logging_enabled) {
        return true;
    }

    csv_snapshot_t snapshots[APP_CSV_BUFFER_LINES];
    size_t count = 0;

    while (count < APP_CSV_BUFFER_LINES && xQueueReceive(s_snapshot_queue, &snapshots[count], 0) == pdTRUE) {
        count++;
    }

    if (count == 0) {
        return true;
    }

    FILE *fp = fopen(SDCARD_LOG_FILENAME, "a");
    if (fp == NULL) {
        ESP_LOGE(TAG, "Unable to open %s for append", SDCARD_LOG_FILENAME);
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        char line[APP_CSV_LINE_MAX_LEN];
        csv_logger_format_line(line, sizeof(line), &snapshots[i]);
        if (fputs(line, fp) == EOF) {
            fclose(fp);
            ESP_LOGE(TAG, "Failed to write CSV batch");
            return false;
        }
    }

    fflush(fp);
    fclose(fp);
    return true;
}

bool csv_logger_set_logging_enabled(bool enabled)
{
    s_logging_enabled = enabled;
    if (!enabled) {
        return csv_logger_flush_pending();
    }

    FILE *fp = fopen(SDCARD_LOG_FILENAME, "w");
    if (fp == NULL) {
        ESP_LOGE(TAG, "Unable to create %s", SDCARD_LOG_FILENAME);
        return false;
    }

    if (fputs(CSV_LOG_HEADER, fp) == EOF) {
        fclose(fp);
        ESP_LOGE(TAG, "Failed to write CSV header");
        return false;
    }

    fclose(fp);
    return true;
}

bool csv_logger_is_logging_enabled(void)
{
    return s_logging_enabled;
}

void csv_logger_set_overflow(uint32_t overflow_count, uint32_t dropped_samples)
{
    s_overflow_count = overflow_count;
    s_dropped_samples = dropped_samples;
}
