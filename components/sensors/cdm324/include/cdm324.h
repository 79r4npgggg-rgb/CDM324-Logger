#pragma once

#include <stdbool.h>
#include <stdint.h>

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
 *   - Local spectral peaks
 */

#define CDM324_STATUS_NORMAL    0U
#define CDM324_STATUS_OVERFLOW  1U
#define CDM324_STATUS_INVALID   2U
#define CDM324_STATUS_NO_SIGNAL 3U

#define CDM324_MAX_PEAKS        5U

typedef struct {
    int32_t frequency_hz;
    float power;
} cdm324_peak_t;

typedef struct {
    uint32_t time_us;

    uint32_t status;

    int32_t aout_dc_mv;
    int32_t aout_rms_mv;
    int32_t aout_pp_mv;

    /*
     * Kept for comparison with the current implementation.
     *
     * This is the strongest FFT bin within the configured
     * search range. It is not yet a validated vehicle-speed
     * estimate.
     */
    int32_t doppler_hz;

    /*
     * Local spectral peak candidates.
     *
     * Peaks are stored in ascending frequency order.
     */
    uint32_t peak_count;

    cdm324_peak_t peaks[CDM324_MAX_PEAKS];

} cdm324_snapshot_t;

bool cdm324_init(void);
bool cdm324_start(void);
bool cdm324_stop(void);

bool cdm324_get_latest_snapshot(
    cdm324_snapshot_t *out_snapshot);