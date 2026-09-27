#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t qre_edges;
    uint32_t esc_edges;
} input_diag_counts_t;

/**
 * Initialize GPIO edge diagnostics for:
 *
 *   QRE1113 : GPIO43
 *   ESC PWM : GPIO44
 *
 * Counts both rising and falling edges.
 */
bool input_diag_init(void);

/**
 * Get current edge counters and optionally clear them.
 *
 * clear_after_read = true:
 *   Return counts accumulated since the previous read,
 *   then reset counters to zero.
 *
 * clear_after_read = false:
 *   Return cumulative counts.
 */
bool input_diag_get_counts(
    input_diag_counts_t *counts,
    bool clear_after_read
);

#ifdef __cplusplus
}
#endif