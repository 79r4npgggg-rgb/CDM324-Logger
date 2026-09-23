#include "spectrum_logger.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "app_config.h"

static const char *TAG = "spectrum_logger";

#define SPECTRUM_LOG_QUEUE_LEN  4U

static QueueHandle_t s_queue = NULL;

static TaskHandle_t s_writer_task = NULL;

static bool s_logging_enabled = false;

static char s_log_filename[32] = {0};


/*
 * ------------------------------------------------------------
 * Binary file format
 * ------------------------------------------------------------
 */

typedef struct __attribute__((packed)) {

    char magic[8];

    uint16_t version;

    uint16_t header_size;

    uint32_t sample_rate_hz;

    uint32_t fft_size;

    uint32_t bin_count;

    uint32_t record_size;

} spectrum_file_header_t;


#define SPECTRUM_MAGIC {'C', 'D', 'M', '3', '2', '4', 'S', 'P'}


static bool spectrum_logger_select_new_filename(void)
{
    for (uint32_t index = 1U;
         index <= 9999U;
         ++index) {

        char filename[32];

        snprintf(
            filename,
            sizeof(filename),
            "/sdcard/SPEC%04lu.BIN",
            (unsigned long)index);


        FILE *fp =
            fopen(filename, "r");


        if (fp == NULL) {

            strncpy(
                s_log_filename,
                filename,
                sizeof(s_log_filename) - 1U);

            s_log_filename[
                sizeof(s_log_filename) - 1U] =
                '\0';


            ESP_LOGI(
                TAG,
                "Selected spectrum file: %s",
                s_log_filename);


            return true;
        }


        fclose(fp);
    }


    ESP_LOGE(
        TAG,
        "No available spectrum filename");


    return false;
}


static bool spectrum_logger_write_header(
    FILE *fp)
{
    spectrum_file_header_t header = {
        .magic = SPECTRUM_MAGIC,
        .version = SPECTRUM_BIN_VERSION,
        .header_size =
            sizeof(spectrum_file_header_t),
        .sample_rate_hz =
            CDM324_ADC_SAMPLE_RATE_HZ,
        .fft_size =
            CDM324_FFT_SIZE,
        .bin_count =
            CDM324_SPECTRUM_BIN_COUNT,
        .record_size =
            sizeof(spectrum_record_t),
    };


    if (fwrite(
            &header,
            sizeof(header),
            1,
            fp) != 1) {

        return false;
    }


    return fflush(fp) == 0;
}


static void spectrum_logger_writer_task(
    void *arg)
{
    (void)arg;

    FILE *fp = NULL;


    while (1) {

        spectrum_record_t record;


        if (xQueueReceive(
                s_queue,
                &record,
                pdMS_TO_TICKS(10)) == pdTRUE) {


            if (fp == NULL) {

                fp =
                    fopen(
                        s_log_filename,
                        "ab");


                if (fp == NULL) {

                    ESP_LOGE(
                        TAG,
                        "Unable to open %s",
                        s_log_filename);

                    continue;
                }
            }


            if (fwrite(
                    &record,
                    sizeof(record),
                    1,
                    fp) != 1) {

                ESP_LOGE(
                    TAG,
                    "Failed to write spectrum");

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


bool spectrum_logger_init(void)
{
    s_queue =
        xQueueCreate(
            SPECTRUM_LOG_QUEUE_LEN,
            sizeof(spectrum_record_t));


    if (s_queue == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create spectrum queue");

        return false;
    }


    s_logging_enabled = false;


    BaseType_t result =
        xTaskCreate(
            spectrum_logger_writer_task,
            "spectrum_writer",
            4096,
            NULL,
            4,
            &s_writer_task);


    if (result != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create spectrum writer task");

        s_writer_task = NULL;

        return false;
    }


    return true;
}


bool spectrum_logger_queue_record(
    const spectrum_record_t *record)
{
    if (record == NULL ||
        s_queue == NULL ||
        !s_logging_enabled) {

        return false;
    }


    return
        xQueueSend(
            s_queue,
            record,
            0) == pdTRUE;
}


bool spectrum_logger_set_logging_enabled(
    bool enabled)
{
    if (enabled) {

        if (!spectrum_logger_select_new_filename()) {
            return false;
        }


        FILE *fp =
            fopen(
                s_log_filename,
                "wb");


        if (fp == NULL) {

            ESP_LOGE(
                TAG,
                "Unable to create %s",
                s_log_filename);

            return false;
        }


        if (!spectrum_logger_write_header(fp)) {

            fclose(fp);

            ESP_LOGE(
                TAG,
                "Failed to write spectrum header");

            return false;
        }


        fclose(fp);


        s_logging_enabled = true;

        return true;
    }


    s_logging_enabled = false;

    return true;
}


bool spectrum_logger_is_logging_enabled(void)
{
    return s_logging_enabled;
}