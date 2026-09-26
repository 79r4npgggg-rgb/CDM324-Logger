#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t high_us;
    uint32_t low_us;
    uint32_t period_us;
    uint16_t duty_permille;
} esc_pwm_measurement_t;

/**
 * @brief Initialize ESC PWM input capture.
 *
 * Uses GPIO44 / D7 and ESP32-S3 RMT RX.
 *
 * @return true on success, false on failure.
 */
bool esc_pwm_init(void);

/**
 * @brief Get the latest ESC PWM measurement.
 *
 * This returns true only when a new measurement is available.
 *
 * @param[out] measurement Latest measurement.
 * @return true if a new measurement was available.
 */
bool esc_pwm_get_latest(esc_pwm_measurement_t *measurement);