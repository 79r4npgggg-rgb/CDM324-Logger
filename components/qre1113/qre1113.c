#include "qre1113.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "board.h"

static const char *TAG = "QRE1113";

/*
 * One reflective mark on the spur gear
 * means one pulse per spur revolution.
 */
#define QRE_PULSES_PER_REV 1U

/*
 * RPM calculation interval.
 */
#define QRE_MEASUREMENT_INTERVAL_MS 100U

static pcnt_unit_handle_t s_pcnt_unit = NULL;
static pcnt_channel_handle_t s_pcnt_channel = NULL;

static qre1113_measurement_t s_latest;
static volatile bool s_new_measurement = false;

static int64_t s_last_measurement_us = 0;


/*
 * Convert spur RPM to motor RPM.
 *
 *      71T spur
 *          ↓
 *      20T pinion
 *
 * Therefore:
 *
 * motor RPM = spur RPM × 71 / 20
 */
static uint32_t qre_spur_to_motor_rpm(uint32_t spur_rpm)
{
    return (uint32_t)(
        ((uint64_t)spur_rpm * QRE_SPUR_TEETH)
        / QRE_PINION_TEETH
    );
}


bool qre1113_init(void)
{
    ESP_LOGI(
        TAG,
        "Initializing QRE1113 on GPIO%d",
        BOARD_GPIO_QRE_INPUT
    );

    /*
     * Configure input GPIO.
     *
     * The actual QRE1113 output circuit may require
     * an external pull-up depending on the implementation.
     */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_QRE_INPUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /*
     * PCNT unit.
     */
    const pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };

    esp_err_t err = pcnt_new_unit(
        &unit_config,
        &s_pcnt_unit
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "pcnt_new_unit failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    /*
     * PCNT channel.
     */
    const pcnt_chan_config_t chan_config = {
        .edge_gpio_num = BOARD_GPIO_QRE_INPUT,
        .level_gpio_num = -1,
    };

    err = pcnt_new_channel(
        s_pcnt_unit,
        &chan_config,
        &s_pcnt_channel
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "pcnt_new_channel failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    /*
     * Count rising edges.
     *
     * Falling edge is ignored.
     */
    err = pcnt_channel_set_edge_action(
        s_pcnt_channel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "pcnt_channel_set_edge_action failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    /*
     * Enable PCNT.
     */
    err = pcnt_unit_enable(s_pcnt_unit);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "pcnt_unit_enable failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    /*
     * Start counting.
     */
    err = pcnt_unit_start(s_pcnt_unit);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "pcnt_unit_start failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    s_last_measurement_us = esp_timer_get_time();

    ESP_LOGI(
        TAG,
        "QRE1113 ready: GPIO%d, %u pulse/rev",
        BOARD_GPIO_QRE_INPUT,
        QRE_PULSES_PER_REV
    );

    return true;
}


bool qre1113_get_latest(qre1113_measurement_t *measurement)
{
    if (measurement == NULL || s_pcnt_unit == NULL) {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();

    const int64_t elapsed_us =
        now_us - s_last_measurement_us;

    if (elapsed_us <
        ((int64_t)QRE_MEASUREMENT_INTERVAL_MS * 1000LL)) {

        return false;
    }

    int count = 0;

    esp_err_t err = pcnt_unit_get_count(
        s_pcnt_unit,
        &count
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "pcnt_unit_get_count failed: %s",
            esp_err_to_name(err)
        );
        return false;
    }

    /*
     * Reset counter for next measurement window.
     */
    pcnt_unit_clear_count(s_pcnt_unit);

    s_last_measurement_us = now_us;

    /*
     * Convert pulse count to spur RPM.
     *
     * RPM =
     *
     * pulses
     * -------- × 60
     * seconds × pulses/rev
     */
    const uint32_t spur_rpm =
        (uint32_t)(
            ((uint64_t)count * 60000000ULL)
            /
            ((uint64_t)elapsed_us * QRE_PULSES_PER_REV)
        );

    const uint32_t motor_rpm =
        qre_spur_to_motor_rpm(spur_rpm);

    s_latest.spur_rpm = spur_rpm;
    s_latest.motor_rpm = motor_rpm;

    *measurement = s_latest;

    return true;
}