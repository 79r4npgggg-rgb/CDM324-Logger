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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;

static bool cdm324_adc_init(void)
{
    const adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    if (adc_oneshot_new_unit(&init_config, &s_adc_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit");
        return false;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    if (adc_oneshot_config_channel(
            s_adc_handle,
            ADC_CHANNEL_0,
            &channel_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure VOUT ADC");
        return false;
    }

    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_0,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    if (adc_cali_create_scheme_curve_fitting(
            &cali_config,
            &s_adc_cali_handle) != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable");
        s_adc_cali_handle = NULL;
    }

    return true;
}

static int32_t cdm324_read_vout_mv(void)
{
    int raw = 0;

    if (s_adc_handle == NULL) {
        return 0;
    }

    if (adc_oneshot_read(
            s_adc_handle,
            ADC_CHANNEL_0,
            &raw) != ESP_OK) {
        return 0;
    }

    int adc_mv = 0;

    if (s_adc_cali_handle != NULL) {
        if (adc_cali_raw_to_voltage(
                s_adc_cali_handle,
                raw,
                &adc_mv) != ESP_OK) {
            return 0;
        }
    } else {
        return 0;
    }

    /*
     * VOUT is divided by 10k / 10k before entering GPIO1.
     * Therefore the original CDM324 VOUT is approximately 2x ADC voltage.
     */
    return adc_mv * 2;
}

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
    .pin_bit_mask = (1ULL << BOARD_GPIO_CDM324_FOUT),
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

    if (gpio_isr_handler_add(BOARD_GPIO_CDM324_FOUT, cdm324_gpio_isr_handler, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler");
        return false;
    }

    if (!cdm324_adc_init()) {
        ESP_LOGE(TAG, "Failed to initialize VOUT ADC");
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
    out_snapshot->vout_mv = cdm324_read_vout_mv();

    return true;
}

QueueHandle_t cdm324_get_event_queue(void)
{
    return s_event_queue;
}
