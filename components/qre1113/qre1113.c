#include "qre1113.h"

#include "driver/gpio.h"

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

/*
 * HIGH duration filter.
 *
 * The QRE output is:
 *
 *   black / non-reflective : HIGH
 *   white reflective tape   : LOW
 *
 * We regard one sufficiently long HIGH period
 * as one valid pulse.
 */
#define QRE_HIGH_FILTER_MIN_US 200U
#define QRE_HIGH_FILTER_MAX_US 500000U

/*
 * Minimum interval between valid pulses.
 *
 * This prevents multiple valid-looking HIGH periods
 * from being counted as separate pulses due to noise.
 */
#define QRE_MIN_INTERVAL_US 500U


/*
 * --------------------------------------------------------------------------
 * GPIO ISR state
 * --------------------------------------------------------------------------
 */

/*
 * Start time of the current HIGH period.
 */
static volatile int64_t s_high_start_us = 0;


/*
 * Diagnostic counters.
 */
static volatile uint32_t s_rise_count = 0;
static volatile uint32_t s_fall_count = 0;

static volatile uint32_t s_high_count = 0;
static volatile uint32_t s_high_confirm_count = 0;
static volatile uint32_t s_high_reject_count = 0;

static volatile uint32_t s_high_min_us = UINT32_MAX;
static volatile uint32_t s_high_max_us = 0;

static volatile uint32_t s_interval_reject_count = 0;


/*
 * Filtered pulse counter.
 *
 * This is the counter used for RPM calculation.
 */
static volatile uint32_t s_filtered_pulse_count = 0;


/*
 * Timestamp of the previous valid pulse.
 */
static volatile int64_t s_last_filtered_pulse_us = 0;


/*
 * Measurement state.
 */
static qre1113_measurement_t s_latest;
static int64_t s_last_measurement_us = 0;


/*
 * --------------------------------------------------------------------------
 * GPIO ISR
 * --------------------------------------------------------------------------
 *
 * HIGH period is measured.
 *
 * Rising edge:
 *     HIGH starts.
 *
 * Falling edge:
 *     HIGH duration is measured.
 *     If the duration is valid, one pulse is counted.
 */
static void IRAM_ATTR qre_gpio_isr_handler(void *arg)
{
    const int level =
        gpio_get_level(BOARD_GPIO_QRE_INPUT);

    const int64_t now_us =
        esp_timer_get_time();


    /*
     * HIGH started.
     */
    if (level == 1) {

        s_rise_count++;

        /*
         * Ignore a rising edge while another HIGH period
         * is already being measured.
         */
        if (s_high_start_us == 0) {
            s_high_start_us = now_us;
        }

        return;
    }


    /*
     * HIGH ended.
     */
    s_fall_count++;

    if (s_high_start_us == 0) {
        return;
    }


    const uint32_t high_duration_us =
        (uint32_t)(now_us - s_high_start_us);

    s_high_count++;


    /*
     * Diagnostic statistics.
     */
    if (high_duration_us < s_high_min_us) {
        s_high_min_us = high_duration_us;
    }

    if (high_duration_us > s_high_max_us) {
        s_high_max_us = high_duration_us;
    }


    /*
     * Check HIGH width.
     */
    if (
        high_duration_us >= QRE_HIGH_FILTER_MIN_US &&
        high_duration_us <= QRE_HIGH_FILTER_MAX_US
    ) {

        /*
         * HIGH duration is valid.
         */
        s_high_confirm_count++;


        /*
         * Make sure enough time has elapsed since
         * the previous valid pulse.
         */
        if (
            s_last_filtered_pulse_us == 0 ||
            (uint64_t)(now_us - s_last_filtered_pulse_us)
                >= QRE_MIN_INTERVAL_US
        ) {

            /*
             * Valid pulse.
             */
            s_filtered_pulse_count++;

            s_last_filtered_pulse_us = now_us;

        } else {

            /*
             * Too close to the previous valid pulse.
             */
            s_interval_reject_count++;
        }

    } else {

        /*
         * HIGH duration was too short or too long.
         */
        s_high_reject_count++;
    }


    /*
     * HIGH period is complete.
     */
    s_high_start_us = 0;
}


/*
 * --------------------------------------------------------------------------
 * Spur RPM -> Motor RPM
 * --------------------------------------------------------------------------
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


/*
 * --------------------------------------------------------------------------
 * ISR diagnostics
 * --------------------------------------------------------------------------
 */
bool qre1113_get_isr_diag(
    qre1113_isr_diag_t *diag,
    bool clear_after_read
)
{
    if (diag == NULL) {
        return false;
    }


    diag->fall_count =
        s_fall_count;

    diag->rise_count =
        s_rise_count;

    diag->low_confirm_count =
        s_high_confirm_count;

    diag->low_reject_count =
        s_high_reject_count;

    diag->low_min_us =
        s_high_min_us;

    diag->low_max_us =
        s_high_max_us;

    diag->low_count =
        s_high_count;

    diag->filtered_pulse_count =
        s_filtered_pulse_count;


    if (clear_after_read) {

        s_fall_count = 0;
        s_rise_count = 0;

        s_high_confirm_count = 0;
        s_high_reject_count = 0;

        s_high_min_us = UINT32_MAX;
        s_high_max_us = 0;

        s_high_count = 0;

        /*
         * Do NOT clear s_filtered_pulse_count here.
         *
         * It is the actual RPM measurement counter and is
         * consumed by qre1113_get_latest().
         */
    }


    return true;
}


/*
 * --------------------------------------------------------------------------
 * Initialization
 * --------------------------------------------------------------------------
 */
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
     * External 10 kΩ pull-up is used.
     *
     * Internal pull-up/down is therefore disabled.
     */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOARD_GPIO_QRE_INPUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));


    /*
     * Install GPIO ISR service.
     */
    esp_err_t err = gpio_install_isr_service(
        ESP_INTR_FLAG_IRAM
    );

    if (
        err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE
    ) {
        ESP_LOGE(
            TAG,
            "gpio_install_isr_service failed: %s",
            esp_err_to_name(err)
        );

        return false;
    }


    /*
     * Register GPIO ISR.
     */
    err = gpio_isr_handler_add(
        BOARD_GPIO_QRE_INPUT,
        qre_gpio_isr_handler,
        NULL
    );

    ESP_LOGI(
        TAG,
        "QRE gpio_isr_handler_add: %s",
        esp_err_to_name(err)
    );

    if (err != ESP_OK) {
        return false;
    }


    /*
     * Initialize measurement state.
     */
    s_high_start_us = 0;

    s_last_filtered_pulse_us = 0;

    s_filtered_pulse_count = 0;

    s_last_measurement_us =
        esp_timer_get_time();


    ESP_LOGI(
        TAG,
        "QRE1113 ready: GPIO%d, %u pulse/rev",
        BOARD_GPIO_QRE_INPUT,
        QRE_PULSES_PER_REV
    );


    return true;
}


/*
 * --------------------------------------------------------------------------
 * Measurement
 * --------------------------------------------------------------------------
 *
 * Every 100 ms:
 *
 *     filtered pulses
 *              × 60
 * RPM = --------------------
 *       elapsed seconds
 *
 * Since QRE_PULSES_PER_REV = 1,
 * one valid HIGH period corresponds to one spur revolution.
 */
bool qre1113_get_latest(
    qre1113_measurement_t *measurement
)
{
    if (measurement == NULL) {
        return false;
    }


    const int64_t now_us =
        esp_timer_get_time();

    const int64_t elapsed_us =
        now_us - s_last_measurement_us;


    if (
        elapsed_us <
        ((int64_t)QRE_MEASUREMENT_INTERVAL_MS * 1000LL)
    ) {
        return false;
    }


    /*
     * Get the number of filtered pulses generated
     * during this measurement interval.
     *
     * Interrupts continue running while this function
     * executes, so briefly disable the GPIO interrupt
     * to make the read-and-clear operation atomic.
     */
    gpio_intr_disable(BOARD_GPIO_QRE_INPUT);

    const uint32_t count =
        s_filtered_pulse_count;

    s_filtered_pulse_count = 0;

    gpio_intr_enable(BOARD_GPIO_QRE_INPUT);


    ESP_LOGI(
        TAG,
        "QRE filtered pulse count=%lu elapsed=%lld us",
        (unsigned long)count,
        elapsed_us
    );


    s_last_measurement_us =
        now_us;


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
            (
                (uint64_t)elapsed_us *
                QRE_PULSES_PER_REV
            )
        );


    /*
     * Convert spur RPM to motor RPM.
     */
    const uint32_t motor_rpm =
        qre_spur_to_motor_rpm(spur_rpm);


    s_latest.spur_rpm =
        spur_rpm;

    s_latest.motor_rpm =
        motor_rpm;


    *measurement =
        s_latest;


    return true;
}