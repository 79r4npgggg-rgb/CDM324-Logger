#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*
 * CDM324 Aout
 *
 * Aout is connected to GPIO1 / ADC1_CH0.
 *
 * The signal is sampled continuously and analyzed as:
 *   - DC level
 *   - AC RMS
 *   - Peak-to-peak
 *   - FFT peak frequency
 */

#define CDM324_STATUS_NORMAL    0U
#define CDM324_STATUS_OVERFLOW  1U
#define CDM324_STATUS_INVALID   2U
#define CDM324_STATUS_NO_SIGNAL 3U

typedef struct {
    uint32_t time_us;

    uint32_t status;

    int32_t aout_dc_mv;
    int32_t aout_rms_mv;
    int32_t aout_pp_mv;

    int32_t doppler_hz;
} cdm324_snapshot_t;

bool cdm324_init(void);
bool cdm324_start(void);
bool cdm324_stop(void);

bool cdm324_get_latest_snapshot(
    cdm324_snapshot_t *out_snapshot);