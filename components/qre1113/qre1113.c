#include "qre1113.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "board.h"

static const char *TAG = "QRE1113";

/*
 * One magnetic / reflective mark
 * means one pulse per spur revolution.
 */
#define QRE_PULSES_PER_REV 1U

/*
 * RPM measurement update interval.
 *
 * The actual RPM is calculated from the time between
 * valid pulses. This interval only determines how often
 * the application receives the latest RPM value.
 */
#define QRE_MEASUREMENT_INTERVAL_MS 100U

/*
 * HIGH duration filter.
 *
 * A sufficiently long HIGH period is treated as one
 * valid pulse.
 *
 * Current experimental value:
 *   200 us ... 500 ms
 */
#define QRE_HIGH_FILTER_MIN_US 200U
#define QRE_HIGH_FILTER_MAX_US 3000000U

/*
 * Minimum interval between valid pulses.
 *
 * This prevents multiple valid-looking pulses caused
 * by noise from being counted separately.
 */
#define QRE_MIN_INTERVAL_US 500U

/*
 * If no valid pulse arrives within this period,
 * the RPM is regarded as zero.
 *
 * 500 ms corresponds to 120 RPM for
 * one pulse per revolution.
 */
#define QRE_RPM_TIMEOUT_US 3000000U


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
 * This counter is consumed by the application
 * measurement function for diagnostics only.
 */
static volatile uint32_t s_filtered_pulse_count = 0;


/*
 * Timestamp of the previous valid pulse.
 *
 * Used for period-based RPM calculation.
 */
static volatile int64_t s_last_filtered_pulse_us = 0;


/*
 * Latest spur RPM calculated from the pulse period.
 */
static volatile uint32_t s_period_spur_rpm = 0;


/*
 * Measurement state.
 */
static qre1113_measurement_t s_latest = {
    .spur_rpm = 0,
    .motor_rpm = 0,
};

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
 *
 * If the HIGH duration is valid, one pulse is accepted.
 *
 * RPM is calculated from the interval between accepted pulses.
 */
static void IRAM_ATTR qre_gpio_isr_handler(void *arg)
{
    (void)arg;

    const int level =
        gpio_get_level(BOARD_GPIO_QRE_INPUT);

    const int64_t now_us =
        esp_timer_get_time();


    /*
     * ------------------------------------------------------
     * HIGH started
     * ------------------------------------------------------
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
     * ------------------------------------------------------
     * HIGH ended
     * ------------------------------------------------------
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
     * ------------------------------------------------------
     * HIGH width filter
     * ------------------------------------------------------
     */
    if (
        high_duration_us >= QRE_HIGH_FILTER_MIN_US &&
        high_duration_us <= QRE_HIGH_FILTER_MAX_US
    ) {

        s_high_confirm_count++;


        /*
         * --------------------------------------------------
         * Minimum interval filter
         * --------------------------------------------------
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


            /*
             * Calculate RPM from the interval between
             * two consecutive valid pulses.
             *
             * One pulse = one spur revolution.
             *
             * RPM = 60,000,000 / interval_us
             */
            if (s_last_filtered_pulse_us != 0) {

                const uint64_t interval_us =
                    (uint64_t)(
                        now_us -
                        s_last_filtered_pulse_us
                    );

                if (interval_us > 0) {

                    s_period_spur_rpm =
                        (uint32_t)(
                            60000000ULL /
                            interval_us
                        );
                }
            }


            /*
             * Save this pulse as the reference
             * for the next revolution.
             */
            s_last_filtered_pulse_us =
                now_us;

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
        (
            (uint64_t)spur_rpm *
            QRE_SPUR_TEETH
        )
        /
        QRE_PINION_TEETH
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
         * Do NOT clear:
         *
         *   s_filtered_pulse_count
         *
         * because it is also used by the RPM
         * measurement function.
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
     * Internal pull-up/down is disabled.
     */
    gpio_config_t io_conf = {
        .pin_bit_mask =
            1ULL << BOARD_GPIO_QRE_INPUT,

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_ANYEDGE,
    };

    ESP_ERROR_CHECK(
        gpio_config(&io_conf)
    );


    /*
     * Install GPIO ISR service.
     */
    esp_err_t err =
        gpio_install_isr_service(
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
    err =
        gpio_isr_handler_add(
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

    s_period_spur_rpm = 0;

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
 * The RPM itself is calculated in the GPIO ISR from
 * the interval between valid pulses.
 *
 * This function periodically returns the latest RPM.
 *
 * If no valid pulse has arrived for QRE_RPM_TIMEOUT_US,
 * RPM is forced to zero.
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
        now_us -
        s_last_measurement_us;


    /*
     * Update at the existing 100 ms application rate.
     */
    if (
        elapsed_us <
        (
            (int64_t)
            QRE_MEASUREMENT_INTERVAL_MS *
            1000LL
        )
    ) {
        return false;
    }


    /*
     * Read the ISR-generated RPM state atomically.
     *
     * The interrupt is disabled only for this very short
     * read operation.
     */
    gpio_intr_disable(
        BOARD_GPIO_QRE_INPUT
    );


    const int64_t last_pulse_us =
        s_last_filtered_pulse_us;

    uint32_t spur_rpm =
        s_period_spur_rpm;


    gpio_intr_enable(
        BOARD_GPIO_QRE_INPUT
    );


    /*
     * ------------------------------------------------------
     * RPM timeout
     * ------------------------------------------------------
     *
     * No valid pulse for 500 ms means stopped.
     */
    if (
        last_pulse_us == 0 ||
        (uint64_t)(
            now_us -
            last_pulse_us
        ) >= QRE_RPM_TIMEOUT_US
    ) {

        spur_rpm = 0;
    }


    /*
     * Convert spur RPM to motor RPM.
     */
    const uint32_t motor_rpm =
        qre_spur_to_motor_rpm(
            spur_rpm
        );


    /*
     * Save latest measurement.
     */
    s_latest.spur_rpm =
        spur_rpm;

    s_latest.motor_rpm =
        motor_rpm;


    *measurement =
        s_latest;


    s_last_measurement_us =
        now_us;


    /*
     * Diagnostic log.
     */
    ESP_LOGI(
        TAG,
        "QRE RPM: SPUR=%lu RPM MOTOR=%lu RPM",
        (unsigned long)spur_rpm,
        (unsigned long)motor_rpm
    );


    return true;
}