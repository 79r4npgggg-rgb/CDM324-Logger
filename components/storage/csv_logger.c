#include "csv_logger.h"

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "app_config.h"
#include "sdcard.h"

static const char *TAG = "csv_logger";

static QueueHandle_t s_snapshot_queue = NULL;

static TaskHandle_t s_writer_task = NULL;

static uint32_t s_overflow_count = 0;
static uint32_t s_dropped_samples = 0;

static bool s_logging_enabled = false;


static void csv_logger_format_line(
    char *out,
    size_t out_size,
    const csv_snapshot_t *snapshot)
{
    if (out == NULL ||
        snapshot == NULL ||
        out_size == 0) {

        return;
    }

    int offset =
        snprintf(
            out,
            out_size,

            "%lu,%ld,%ld,%ld,%ld,%lu,%lu",

            (unsigned long)snapshot->time_us,

            (long)snapshot->aout_dc_mv,

            (long)snapshot->aout_rms_mv,

            (long)snapshot->aout_pp_mv,

            (long)snapshot->doppler_hz,

            (unsigned long)snapshot->status,

            (unsigned long)snapshot->peak_count);

    if (offset < 0 ||
        (size_t)offset >= out_size) {

        return;
    }

    for (uint32_t i = 0;
         i < CSV_MAX_PEAKS;
         ++i) {

        const int written =
            snprintf(
                out + offset,
                out_size - (size_t)offset,

                ",%ld,%.6e",

                (long)snapshot->peaks[i].frequency_hz,

                (double)snapshot->peaks[i].power);

        if (written < 0 ||
            (size_t)written >=
                out_size - (size_t)offset) {

            return;
        }

        offset += written;
    }

    if ((size_t)offset + 2U < out_size) {

        out[offset++] = '\n';
        out[offset] = '\0';
    }
}


static void csv_logger_writer_task(
    void *arg)
{
    (void)arg;

    FILE *fp = NULL;


    while (1) {

        csv_snapshot_t snapshot;


        if (xQueueReceive(
                s_snapshot_queue,
                &snapshot,
                pdMS_TO_TICKS(10)) == pdTRUE) {


            if (fp == NULL) {

                fp =
                    fopen(
                        SDCARD_LOG_FILENAME,
                        "a");


                if (fp == NULL) {

                    ESP_LOGE(
                        TAG,
                        "Unable to open %s for append",
                        SDCARD_LOG_FILENAME);

                    continue;
                }
            }


            char line[
                APP_CSV_LINE_MAX_LEN];


            csv_logger_format_line(
                line,
                sizeof(line),
                &snapshot);


            if (fputs(
                    line,
                    fp) == EOF) {

                ESP_LOGE(
                    TAG,
                    "Failed to write CSV");

                fclose(fp);

                fp = NULL;

                continue;
            }


            fflush(fp);


        } else if (
            !s_logging_enabled &&
            fp != NULL) {

            fclose(fp);

            fp = NULL;
        }
    }
}


bool csv_logger_init(void)
{
    const app_config_t *config =
        app_config_get();


    s_snapshot_queue =
        xQueueCreate(
            config->csv_buffer_lines,
            sizeof(csv_snapshot_t));


    if (s_snapshot_queue == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create CSV queue");

        return false;
    }


    s_logging_enabled = false;


    BaseType_t result =
        xTaskCreate(
            csv_logger_writer_task,
            "csv_writer",
            4096,
            NULL,
            4,
            &s_writer_task);


    if (result != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create CSV writer task");

        s_writer_task = NULL;

        return false;
    }


    return true;
}


bool csv_logger_queue_snapshot(
    const csv_snapshot_t *snapshot)
{
    if (snapshot == NULL ||
        s_snapshot_queue == NULL ||
        !s_logging_enabled) {

        return false;
    }


    if (xQueueSend(
            s_snapshot_queue,
            snapshot,
            0) != pdTRUE) {

        s_dropped_samples++;

        return false;
    }


    return true;
}


bool csv_logger_set_logging_enabled(
    bool enabled)
{
    if (enabled) {

        FILE *fp =
            fopen(
                SDCARD_LOG_FILENAME,
                "w");


        if (fp == NULL) {

            ESP_LOGE(
                TAG,
                "Unable to create %s",
                SDCARD_LOG_FILENAME);

            return false;
        }


        if (fputs(
                CSV_LOG_HEADER,
                fp) == EOF) {

            fclose(fp);

            ESP_LOGE(
                TAG,
                "Failed to write CSV header");

            return false;
        }


        fclose(fp);


        s_logging_enabled = true;

        return true;
    }


    s_logging_enabled = false;

    return true;
}


bool csv_logger_is_logging_enabled(void)
{
    return s_logging_enabled;
}


void csv_logger_set_overflow(
    uint32_t overflow_count,
    uint32_t dropped_samples)
{
    s_overflow_count =
        overflow_count;

    s_dropped_samples =
        dropped_samples;
}