#include "cdm324.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"

static const char *TAG = "cdm324";

/*
 * ============================================================
 * ADC / FFT configuration
 * ============================================================
 */

/*
 * Aout is connected to:
 *
 *   GPIO1 / ADC1_CH0
 */
#define CDM324_ADC_CHANNEL       ADC_CHANNEL_0

/*
 * ADC sampling frequency.
 *
 * 20 kHz gives:
 *
 *   Nyquist = 10 kHz
 *
 * and is more than sufficient for the current CDM324
 * Doppler-frequency investigation.
 */
#define CDM324_ADC_SAMPLE_RATE_HZ    20000U

/*
 * FFT size.
 *
 * 2048 samples at 20 kHz:
 *
 *   2048 / 20000 = 102.4 ms
 *
 * Frequency resolution:
 *
 *   20000 / 2048 = 9.765625 Hz
 *
 * This is intentionally modest for the first implementation.
 */
#define CDM324_FFT_SIZE           2048U

#define CDM324_ADC_FRAME_SIZE     512U
#define CDM324_ADC_POOL_SIZE      4096U

/*
 * Ignore the very-low-frequency bins when searching for
 * the Doppler peak.
 *
 * Bin width is approximately 9.77 Hz.
 *
 * Starting at bin 2 means approximately 19.5 Hz.
 */
#define CDM324_FFT_MIN_BIN        2U

/*
 * Aout is divided before GPIO1.
 *
 * Current hardware:
 *
 *     Aout ---- 10k ---- GPIO1 ---- 10k ---- GND
 *
 * Therefore:
 *
 *     original Aout = ADC voltage * 2
 */
#define CDM324_AOUT_DIVIDER_NUMERATOR    2U

/*
 * FFT analysis task period.
 *
 * One FFT frame contains approximately 102.4 ms of data.
 */
#define CDM324_ANALYSIS_TASK_STACK       8192U
#define CDM324_ANALYSIS_TASK_PRIORITY    5U


/*
 * ============================================================
 * ADC state
 * ============================================================
 */

static adc_continuous_handle_t s_adc_handle = NULL;

static adc_cali_handle_t s_adc_cali_handle = NULL;

static TaskHandle_t s_analysis_task = NULL;

static volatile bool s_running = false;


/*
 * Latest analysis result.
 *
 * Access is intentionally simple for this first implementation.
 * The analysis task writes the structure and the main task reads it.
 */
static cdm324_snapshot_t s_latest_snapshot;


/*
 * FFT data.
 *
 * ESP-DSP expects complex data:
 *
 *   real, imag, real, imag, ...
 *
 * Therefore:
 *
 *   2048 complex samples = 4096 floats
 */
static float s_fft_data[CDM324_FFT_SIZE * 2];


/*
 * Hann window.
 */
static float s_window[CDM324_FFT_SIZE];


/*
 * ADC samples converted to mV.
 */
static float s_samples_mv[CDM324_FFT_SIZE];


/*
 * ============================================================
 * Forward declarations
 * ============================================================
 */

static bool cdm324_adc_init(void);
static bool cdm324_fft_init(void);

static bool cdm324_read_frame(
    uint16_t *samples,
    size_t sample_count);

static void cdm324_analyze_frame(
    const uint16_t *samples);

static void cdm324_analysis_task(
    void *arg);


/*
 * ============================================================
 * ADC initialization
 * ============================================================
 */

static bool cdm324_adc_init(void)
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = CDM324_ADC_POOL_SIZE,
        .conv_frame_size = CDM324_ADC_FRAME_SIZE,
    };

    esp_err_t ret =
        adc_continuous_new_handle(
            &adc_config,
            &s_adc_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "adc_continuous_new_handle failed: %s",
            esp_err_to_name(ret));

        return false;
    }


    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = CDM324_ADC_CHANNEL,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };


    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = &adc_pattern,
        .sample_freq_hz =
            CDM324_ADC_SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };


    ret =
        adc_continuous_config(
            s_adc_handle,
            &dig_cfg);

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "adc_continuous_config failed: %s",
            esp_err_to_name(ret));

        adc_continuous_deinit(s_adc_handle);
        s_adc_handle = NULL;

        return false;
    }


    /*
     * ADC calibration.
     *
     * Calibration is used only for converting raw ADC
     * values to millivolts. The ADC itself remains in
     * continuous mode.
     */
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = CDM324_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };


    ret =
        adc_cali_create_scheme_curve_fitting(
            &cali_config,
            &s_adc_cali_handle);

    if (ret != ESP_OK) {
        ESP_LOGW(
            TAG,
            "ADC calibration unavailable: %s",
            esp_err_to_name(ret));

        s_adc_cali_handle = NULL;
    }


    ESP_LOGI(
        TAG,
        "Aout ADC initialized: GPIO1 / ADC1_CH0 / %u Hz",
        CDM324_ADC_SAMPLE_RATE_HZ);

    return true;
}


/*
 * ============================================================
 * FFT initialization
 * ============================================================
 */

static bool cdm324_fft_init(void)
{
    /*
     * Generate Hann window.
     */
    for (size_t i = 0; i < CDM324_FFT_SIZE; ++i) {

        s_window[i] =
            0.5f *
            (
                1.0f -
                cosf(
                    2.0f *
                    (float)M_PI *
                    (float)i /
                    (float)(CDM324_FFT_SIZE - 1U)
                )
            );
    }


    /*
     * ESP-DSP FFT table initialization.
     */
    extern esp_err_t dsps_fft2r_init_fc32(
        float *fft_table_buff,
        int table_size);

    esp_err_t ret =
        dsps_fft2r_init_fc32(
            NULL,
            CDM324_FFT_SIZE);

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "FFT initialization failed: %d",
            (int)ret);

        return false;
    }


    ESP_LOGI(
        TAG,
        "FFT initialized: N=%u, resolution=%.3f Hz",
        CDM324_FFT_SIZE,
        (float)CDM324_ADC_SAMPLE_RATE_HZ /
            (float)CDM324_FFT_SIZE);

    return true;
}


/*
 * ============================================================
 * ADC frame acquisition
 * ============================================================
 */

static bool cdm324_read_frame(
    uint16_t *samples,
    size_t sample_count)
{
    if (samples == NULL ||
        sample_count != CDM324_FFT_SIZE ||
        s_adc_handle == NULL) {

        return false;
    }


    size_t sample_index = 0;

    uint8_t result_buffer[CDM324_ADC_FRAME_SIZE];


    while (sample_index < sample_count) {

        uint32_t ret_num = 0;

        esp_err_t ret =
            adc_continuous_read(
                s_adc_handle,
                result_buffer,
                sizeof(result_buffer),
                &ret_num,
                1000);


        if (ret == ESP_ERR_TIMEOUT) {
            continue;
        }


        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "ADC read failed: %s",
                esp_err_to_name(ret));

            return false;
        }


        for (
            size_t offset = 0;
            offset < ret_num;
            offset += SOC_ADC_DIGI_RESULT_BYTES
        ) {

            adc_digi_output_data_t *data =
                (adc_digi_output_data_t *)
                    &result_buffer[offset];


            /*
             * ESP32-S3 uses TYPE1 format.
             */
            if (data->type1.unit != ADC_UNIT_1) {
                continue;
            }


            if (data->type1.channel !=
                CDM324_ADC_CHANNEL) {

                continue;
            }


            if (sample_index <
                sample_count) {

                samples[sample_index++] =
                    data->type1.data;
            }
        }
    }


    return true;
}


/*
 * ============================================================
 * Analyze one FFT frame
 * ============================================================
 */

static void cdm324_analyze_frame(
    const uint16_t *samples)
{
    if (samples == NULL) {
        return;
    }


    float sum_mv = 0.0f;

    float min_mv = 1000000.0f;
    float max_mv = -1000000.0f;


    /*
     * --------------------------------------------------------
     * Convert ADC values to mV
     * --------------------------------------------------------
     */

    for (size_t i = 0;
         i < CDM324_FFT_SIZE;
         ++i) {

        int voltage_mv = 0;

        if (s_adc_cali_handle != NULL) {

            if (adc_cali_raw_to_voltage(
                    s_adc_cali_handle,
                    samples[i],
                    &voltage_mv) != ESP_OK) {

                voltage_mv = 0;
            }

        } else {

            /*
             * Fallback approximation.
             *
             * This is only used if calibration is unavailable.
             */
            voltage_mv =
                ((int)samples[i] * 3100) / 4095;
        }


        /*
         * Restore the original Aout voltage
         * before the 1:2 divider.
         */
        voltage_mv *=
            CDM324_AOUT_DIVIDER_NUMERATOR;


        s_samples_mv[i] =
            (float)voltage_mv;


        sum_mv +=
            s_samples_mv[i];


        if (s_samples_mv[i] < min_mv) {
            min_mv = s_samples_mv[i];
        }

        if (s_samples_mv[i] > max_mv) {
            max_mv = s_samples_mv[i];
        }
    }


    const float dc_mv =
        sum_mv /
        (float)CDM324_FFT_SIZE;


    /*
     * --------------------------------------------------------
     * AC RMS
     * --------------------------------------------------------
     */

    float sum_square = 0.0f;


    for (size_t i = 0;
         i < CDM324_FFT_SIZE;
         ++i) {

        const float ac =
            s_samples_mv[i] - dc_mv;

        sum_square +=
            ac * ac;
    }


    const float rms_mv =
        sqrtf(
            sum_square /
            (float)CDM324_FFT_SIZE);


    const float pp_mv =
        max_mv - min_mv;


    /*
     * --------------------------------------------------------
     * Prepare FFT input
     * --------------------------------------------------------
     *
     * Remove DC component first.
     *
     * Apply Hann window.
     */
    for (size_t i = 0;
         i < CDM324_FFT_SIZE;
         ++i) {

        const float ac =
            s_samples_mv[i] - dc_mv;


        s_fft_data[i * 2U] =
            ac * s_window[i];

        s_fft_data[i * 2U + 1U] =
            0.0f;
    }


    /*
     * --------------------------------------------------------
     * FFT
     * --------------------------------------------------------
     */

    extern esp_err_t dsps_fft2r_fc32(
        float *data,
        int N);


    extern esp_err_t dsps_bit_rev_fc32(
        float *data,
        int N);


    esp_err_t ret =
        dsps_fft2r_fc32(
            s_fft_data,
            CDM324_FFT_SIZE);


    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "FFT failed: %d",
            (int)ret);

        return;
    }


    ret =
        dsps_bit_rev_fc32(
            s_fft_data,
            CDM324_FFT_SIZE);


    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "FFT bit reverse failed: %d",
            (int)ret);

        return;
    }


    /*
     * --------------------------------------------------------
     * Find strongest positive-frequency bin
     * --------------------------------------------------------
     */

    float max_power = 0.0f;
    size_t max_bin = 0;


    for (
        size_t bin = CDM324_FFT_MIN_BIN;
        bin < CDM324_FFT_SIZE / 2U;
        ++bin
    ) {

        const float real =
            s_fft_data[bin * 2U];

        const float imag =
            s_fft_data[bin * 2U + 1U];


        const float power =
            real * real +
            imag * imag;


        if (power > max_power) {

            max_power = power;
            max_bin = bin;
        }
    }


    const float bin_hz =
        (float)CDM324_ADC_SAMPLE_RATE_HZ /
        (float)CDM324_FFT_SIZE;


    const float doppler_hz =
        (float)max_bin * bin_hz;


    /*
     * --------------------------------------------------------
     * Store result
     * --------------------------------------------------------
     */

    s_latest_snapshot.time_us =
        (uint32_t)esp_timer_get_time();

    s_latest_snapshot.status =
        CDM324_STATUS_NORMAL;

    s_latest_snapshot.aout_dc_mv =
        (int32_t)lroundf(dc_mv);

    s_latest_snapshot.aout_rms_mv =
        (int32_t)lroundf(rms_mv);

    s_latest_snapshot.aout_pp_mv =
        (int32_t)lroundf(pp_mv);

    s_latest_snapshot.doppler_hz =
        (int32_t)lroundf(doppler_hz);
}


/*
 * ============================================================
 * Analysis task
 * ============================================================
 */

static void cdm324_analysis_task(
    void *arg)
{
    (void)arg;

    uint16_t samples[CDM324_FFT_SIZE];


    ESP_LOGI(
        TAG,
        "Aout analysis task started");


    while (s_running) {

        if (!cdm324_read_frame(
                samples,
                CDM324_FFT_SIZE)) {

            s_latest_snapshot.status =
                CDM324_STATUS_INVALID;

            vTaskDelay(
                pdMS_TO_TICKS(10));

            continue;
        }


        cdm324_analyze_frame(samples);
    }


    vTaskDelete(NULL);
}


/*
 * ============================================================
 * Public API
 * ============================================================
 */

bool cdm324_init(void)
{
    memset(
        &s_latest_snapshot,
        0,
        sizeof(s_latest_snapshot));


    /*
     * GPIO1 is ADC input.
     *
     * No digital GPIO configuration is necessary.
     */
    gpio_set_direction(
        BOARD_GPIO_CDM324_AOUT,
        GPIO_MODE_DISABLE);


    if (!cdm324_adc_init()) {
        ESP_LOGE(
            TAG,
            "Aout ADC initialization failed");

        return false;
    }


    if (!cdm324_fft_init()) {
        ESP_LOGE(
            TAG,
            "FFT initialization failed");

        return false;
    }


    ESP_LOGI(
        TAG,
        "CDM324 Aout analyzer initialized");

    return true;
}


bool cdm324_start(void)
{
    if (s_running) {
        return true;
    }


    esp_err_t ret =
        adc_continuous_start(
            s_adc_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "ADC start failed: %s",
            esp_err_to_name(ret));

        return false;
    }


    s_running = true;


    BaseType_t task_ret =
        xTaskCreate(
            cdm324_analysis_task,
            "cdm324_analysis",
            CDM324_ANALYSIS_TASK_STACK,
            NULL,
            CDM324_ANALYSIS_TASK_PRIORITY,
            &s_analysis_task);


    if (task_ret != pdPASS) {

        s_running = false;

        adc_continuous_stop(
            s_adc_handle);

        ESP_LOGE(
            TAG,
            "Failed to create analysis task");

        return false;
    }


    return true;
}


bool cdm324_stop(void)
{
    if (!s_running) {
        return true;
    }


    s_running = false;


    if (s_analysis_task != NULL) {

        /*
         * The task will terminate after the current
         * ADC frame has been processed.
         */
        vTaskDelay(
            pdMS_TO_TICKS(20));

        s_analysis_task = NULL;
    }


    esp_err_t ret =
        adc_continuous_stop(
            s_adc_handle);


    if (ret != ESP_OK) {

        ESP_LOGW(
            TAG,
            "ADC stop failed: %s",
            esp_err_to_name(ret));
    }


    return true;
}


bool cdm324_get_latest_snapshot(
    cdm324_snapshot_t *out_snapshot)
{
    if (out_snapshot == NULL) {
        return false;
    }


    *out_snapshot =
        s_latest_snapshot;


    return true;
}