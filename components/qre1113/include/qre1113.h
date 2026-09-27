#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Motor / spur gear relationship
 *
 * Motor pinion : 20 teeth
 * Spur gear    : 71 teeth
 *
 * QRE1113 measures the spur gear rotation.
 */

#define QRE_PINION_TEETH 20U
#define QRE_SPUR_TEETH   71U

typedef struct {
    uint32_t spur_rpm;
    uint32_t motor_rpm;
} qre1113_measurement_t;

/**
 * @brief Initialize QRE1113 input.
 *
 * QRE1113 is connected to GPIO43 / D6.
 */
bool qre1113_init(void);

/**
 * @brief Get the latest RPM measurement.
 *
 * Returns true only when a new measurement is available.
 */
bool qre1113_get_latest(qre1113_measurement_t *measurement);

typedef struct {
    uint32_t fall_count;
    uint32_t rise_count;

    uint32_t low_confirm_count;
    uint32_t low_reject_count;

    uint32_t filtered_pulse_count;

    uint32_t low_min_us;
    uint32_t low_max_us;
    uint32_t low_count;
} qre1113_isr_diag_t;

bool qre1113_get_isr_diag(
    qre1113_isr_diag_t *diag,
    bool clear_after_read
);