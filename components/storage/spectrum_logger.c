#include "spectrum_logger.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "cdm324.h"

#define TAG "spectrum_logger"

#define SPECTRUM_WRITER_TASK_STACK    4096U
#define SPECTRUM_WRITER_TASK_PRIORITY 4U

#define SPECTRUM_FILENAME_MAX         32U
#define SPECTRUM_FILENAME_MAX_INDEX   9999U

typedef struct __attribute__((packed)) {
    char magic[8];
    uint16_t version;
    uint16_t header_size;
    uint32_t sample_rate_hz;
    uint32_t fft_size;
    uint32_t bin_count;
    uint32_t record_size;
} spectrum_file_header_t;

#define SPECTRUM_MAGIC \
    {'C', 'D', 'M', '3', '2', '4', 'S', 'P'}

static TaskHandle_t s_writer_task = NULL;

static SemaphoreHandle_t s_buffer_mutex = NULL;

static bool s_logging_enabled = false;

static char s_log_filename[
    SPECTRUM_FILENAME_MAX] = {0};

static uint32_t s_pending_time_us = 0U;

static float s_spectrum_buffer[
    CDM324_SPECTRUM_BIN_COUNT];


/*
 * ============================================================
 * File handling
 * ============================================================
 */

static bool spectrum_logger_select_new_filename(void)
{
    for (uint32_t index = 1U;
         index <= SPECTRUM_FILENAME_MAX_INDEX;
         ++index) {

        char filename[SPECTRUM_FILENAME_MAX];

        snprintf(
            filename,
            sizeof(filename),
            "/sdcard/SPEC%04lu.BIN",
            (unsigned long)index
        );

        FILE *fp = fopen(filename, "rb");

        if (fp == NULL) {

            strncpy(
                s_log_filename,
                filename,
                sizeof(s_log_filename) - 1U
            );

            s_log_filename[
                sizeof(s_log_filename) - 1U] = '\0';

            ESP_LOGI(
                TAG,
                "Selected spectrum log file: %s",
                s_log_filename
            );

            return true;
        }

        fclose(fp);
    }

    ESP_LOGE(
        TAG,
        "No available spectrum log filename"
    );

    return false;
}


static bool spectrum_logger_write_header(void)
{
    FILE *fp = fopen(
        s_log_filename,
        "wb"
    );

    if (fp == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create spectrum log file: %s",
            s_log_filename
        );

        return false;
    }

    const spectrum_file_header_t header = {
        .magic = SPECTRUM_MAGIC,
        .version = 1U,
        .header_size =
            sizeof(spectrum_file_header_t),
        .sample_rate_hz =
            CDM324_ADC_SAMPLE_RATE_HZ,
        .fft_size =
            CDM324_FFT_SIZE,
        .bin_count =
            CDM324_SPECTRUM_BIN_COUNT,
        .record_size =
            sizeof(uint32_t) +
            sizeof(float) *
            CDM324_SPECTRUM_BIN_COUNT,
    };

    const size_t written =
        fwrite(
            &header,
            sizeof(header),
            1U,
            fp
        );

    fclose(fp);

    if (written != 1U) {

        ESP_LOGE(
            TAG,
            "Failed to write spectrum file header"
        );

        return false;
    }

    ESP_LOGI(
        TAG,
        "Spectrum file header written: %s",
        s_log_filename
    );

    return true;
}


/*
 * ============================================================
 * Writer task
 * ============================================================
 */

static void spectrum_logger_writer_task(
    void *arg)
{
    (void)arg;

    while (true) {

        /*
         * Wait until app_main submits a new spectrum.
         */
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        if (!s_logging_enabled) {
            continue;
        }

        /*
         * Keep the shared buffer locked while the
         * complete spectrum record is written.
         *
         * This prevents app_main from overwriting
         * the spectrum while fwrite() is using it.
         */
        if (xSemaphoreTake(
                s_buffer_mutex,
                portMAX_DELAY) != pdTRUE) {

            continue;
        }

        FILE *fp = fopen(
            s_log_filename,
            "ab"
        );

        if (fp == NULL) {

            ESP_LOGE(
                TAG,
                "Failed to open spectrum log file: %s",
                s_log_filename
            );

            xSemaphoreGive(
                s_buffer_mutex
            );

            continue;
        }

        const size_t time_written =
            fwrite(
                &s_pending_time_us,
                sizeof(s_pending_time_us),
                1U,
                fp
            );

        const size_t spectrum_written =
            fwrite(
                s_spectrum_buffer,
                sizeof(float),
                CDM324_SPECTRUM_BIN_COUNT,
                fp
            );

        fflush(fp);
        fclose(fp);

        xSemaphoreGive(
            s_buffer_mutex
        );

        if (time_written != 1U ||
            spectrum_written !=
                CDM324_SPECTRUM_BIN_COUNT) {

            ESP_LOGE(
                TAG,
                "Failed to write spectrum record"
            );
        }
    }
}


/*
 * ============================================================
 * Initialization
 * ============================================================
 */

bool spectrum_logger_init(void)
{
    if (s_writer_task != NULL) {
        return true;
    }

    s_buffer_mutex =
        xSemaphoreCreateMutex();

    if (s_buffer_mutex == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create spectrum buffer mutex"
        );

        return false;
    }

    BaseType_t result =
        xTaskCreate(
            spectrum_logger_writer_task,
            "spectrum_writer",
            SPECTRUM_WRITER_TASK_STACK,
            NULL,
            SPECTRUM_WRITER_TASK_PRIORITY,
            &s_writer_task
        );

    if (result != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create spectrum writer task"
        );

        vSemaphoreDelete(
            s_buffer_mutex
        );

        s_buffer_mutex = NULL;

        s_writer_task = NULL;

        return false;
    }

    ESP_LOGI(
        TAG,
        "Spectrum logger initialized"
    );

    return true;
}


/*
 * ============================================================
 * Logging control
 * ============================================================
 */

bool spectrum_logger_set_logging_enabled(
    bool enabled)
{
    if (enabled == s_logging_enabled) {
        return true;
    }

    if (enabled) {

        if (s_writer_task == NULL ||
            s_buffer_mutex == NULL) {

            ESP_LOGE(
                TAG,
                "Spectrum logger is not initialized"
            );

            return false;
        }

        if (!spectrum_logger_select_new_filename()) {
            return false;
        }

        if (!spectrum_logger_write_header()) {
            return false;
        }

        /*
         * Discard any pending writer notification
         * from the previous logging session.
         */
        ulTaskNotifyTake(
            pdTRUE,
            0
        );

        s_logging_enabled = true;

        ESP_LOGI(
            TAG,
            "Spectrum logging enabled"
        );

        return true;
    }

    s_logging_enabled = false;

    ESP_LOGI(
        TAG,
        "Spectrum logging disabled"
    );

    return true;
}


bool spectrum_logger_is_logging_enabled(void)
{
    return s_logging_enabled;
}


/*
 * ============================================================
 * Submit latest spectrum
 * ============================================================
 */

bool spectrum_logger_submit_latest(void)
{
    if (!s_logging_enabled) {
        return false;
    }

    if (s_writer_task == NULL ||
        s_buffer_mutex == NULL) {

        return false;
    }

    uint32_t spectrum_time_us = 0U;

    /*
     * Get the latest FFT spectrum directly into
     * the logger's single shared buffer.
     */
    if (xSemaphoreTake(
            s_buffer_mutex,
            portMAX_DELAY) != pdTRUE) {

        return false;
    }

    const bool success =
        cdm324_get_latest_spectrum(
            s_spectrum_buffer,
            CDM324_SPECTRUM_BIN_COUNT,
            &spectrum_time_us
        );

    if (success) {
        s_pending_time_us =
            spectrum_time_us;
    }

    xSemaphoreGive(
        s_buffer_mutex
    );

    if (!success) {
        return false;
    }

    /*
     * Tell the writer task that a new spectrum
     * is available.
     *
     * No spectrum data is copied here.
     */
    xTaskNotifyGive(
        s_writer_task
    );

    return true;
}