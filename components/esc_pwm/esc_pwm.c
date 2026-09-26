#include "esc_pwm.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board.h"

static const char *TAG = "ESC_PWM";

/*
 * RMT resolution:
 * 1 tick = 1 us
 */
#define ESC_PWM_RMT_RESOLUTION_HZ 1000000

/*
 * RC PWM is normally around:
 *
 *   period : 20 ms
 *   high   : 1.0 - 2.0 ms
 *
 * Capture several symbols at once.
 */
#define ESC_PWM_RX_SYMBOLS 8

typedef struct {
    uint32_t high_us;
    uint32_t low_us;
} esc_pwm_raw_capture_t;

static rmt_channel_handle_t s_rx_channel = NULL;
static QueueHandle_t s_capture_queue = NULL;

static rmt_symbol_word_t s_rx_symbols[ESC_PWM_RX_SYMBOLS];

static esc_pwm_measurement_t s_latest;
static volatile bool s_new_measurement = false;

static portMUX_TYPE s_data_spinlock = portMUX_INITIALIZER_UNLOCKED;


/*
 * RMT RX callback.
 *
 * Keep this callback short.
 * No logging, no calculations, no blocking operations here.
 */
static bool IRAM_ATTR esc_pwm_rmt_rx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    QueueHandle_t queue = (QueueHandle_t)user_data;

    if (edata->num_symbols > 0) {
        const rmt_symbol_word_t *symbol = &edata->received_symbols[0];

        esc_pwm_raw_capture_t capture;

        /*
         * RC PWM normally starts HIGH.
         *
         * Handle either polarity just in case.
         */
        if (symbol->level0 == 1) {
            capture.high_us = symbol->duration0;
            capture.low_us  = symbol->duration1;
        } else {
            capture.low_us  = symbol->duration0;
            capture.high_us = symbol->duration1;
        }

        xQueueSendFromISR(
            queue,
            &capture,
            &high_task_wakeup
        );
    }

    return high_task_wakeup == pdTRUE;
}


static void esc_pwm_capture_task(void *arg)
{
    (void)arg;

    esc_pwm_raw_capture_t capture;

    /*
     * A complete RC PWM pulse is roughly:
     *
     *   HIGH 1-2 ms
     *   LOW  18-19 ms
     *
     * Allow up to 25 ms for one level.
     */
    const rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 25000000,
    };

    while (true) {

        if (xQueueReceive(
                s_capture_queue,
                &capture,
                portMAX_DELAY) == pdTRUE) {

            const uint32_t period_us =
                capture.high_us + capture.low_us;

            uint16_t duty_permille = 0;

            if (period_us > 0) {
                duty_permille =
                    (uint16_t)(
                        ((uint64_t)capture.high_us * 1000ULL)
                        / period_us
                    );
            }

            /*
             * Update shared result.
             */
            portENTER_CRITICAL(&s_data_spinlock);

            s_latest.high_us = capture.high_us;
            s_latest.low_us = capture.low_us;
            s_latest.period_us = period_us;
            s_latest.duty_permille = duty_permille;

            s_new_measurement = true;

            portEXIT_CRITICAL(&s_data_spinlock);

            /*
             * Start the next RMT reception.
             */
            esp_err_t err = rmt_receive(
                s_rx_channel,
                s_rx_symbols,
                sizeof(s_rx_symbols),
                &receive_config
            );

            if (err != ESP_OK) {
                ESP_LOGE(TAG,
                         "rmt_receive() failed: %s",
                         esp_err_to_name(err));
            }
        }
    }
}


bool esc_pwm_init(void)
{
    ESP_LOGI(TAG,
             "Initializing ESC PWM input on GPIO%d",
             BOARD_GPIO_ESC_PWM);

    /*
     * Queue for raw RMT captures.
     */
    s_capture_queue = xQueueCreate(
        4,
        sizeof(esc_pwm_raw_capture_t)
    );

    if (s_capture_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create capture queue");
        return false;
    }

    /*
     * RMT RX channel.
     */
    const rmt_rx_channel_config_t rx_channel_config = {
        .gpio_num = BOARD_GPIO_ESC_PWM,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = ESC_PWM_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .flags.invert_in = false,
        .flags.with_dma = false,
    };

    esp_err_t err = rmt_new_rx_channel(
        &rx_channel_config,
        &s_rx_channel
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "rmt_new_rx_channel() failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    /*
     * Register callback.
     */
    const rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = esc_pwm_rmt_rx_done_callback,
    };

    err = rmt_rx_register_event_callbacks(
        s_rx_channel,
        &callbacks,
        s_capture_queue
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "rmt_rx_register_event_callbacks() failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    /*
     * Enable RMT.
     */
    err = rmt_enable(s_rx_channel);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "rmt_enable() failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    /*
     * Start the first reception.
     */
    const rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 25000000,
    };

    err = rmt_receive(
        s_rx_channel,
        s_rx_symbols,
        sizeof(s_rx_symbols),
        &receive_config
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Initial rmt_receive() failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    /*
     * Processing task.
     */
    BaseType_t task_ok = xTaskCreate(
        esc_pwm_capture_task,
        "esc_pwm",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESC PWM task");
        return false;
    }

    ESP_LOGI(TAG,
             "ESC PWM ready: GPIO%d",
             BOARD_GPIO_ESC_PWM);

    return true;
}


bool esc_pwm_get_latest(esc_pwm_measurement_t *measurement)
{
    if (measurement == NULL) {
        return false;
    }

    bool available = false;

    portENTER_CRITICAL(&s_data_spinlock);

    if (s_new_measurement) {

        memcpy(
            measurement,
            &s_latest,
            sizeof(esc_pwm_measurement_t)
        );

        s_new_measurement = false;
        available = true;
    }

    portEXIT_CRITICAL(&s_data_spinlock);

    return available;
}