#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CDM324_STATUS_NORMAL    0U
#define CDM324_STATUS_OVERFLOW  1U
#define CDM324_STATUS_INVALID   2U
#define CDM324_STATUS_NO_SIGNAL 3U

#define CDM324_MAX_PEAKS        5U

#define CDM324_ADC_SAMPLE_RATE_HZ  20000U
#define CDM324_FFT_SIZE             4096U

/*
 * Positive-frequency spectrum including:
 *
 *   DC      : bin 0
 *   Nyquist : bin 2048
 */
#define CDM324_SPECTRUM_BIN_COUNT \
    (CDM324_FFT_SIZE / 2U + 1U)

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

    int32_t doppler_hz;

    uint32_t peak_count;

    cdm324_peak_t peaks[CDM324_MAX_PEAKS];

} cdm324_snapshot_t;

bool cdm324_init(void);
bool cdm324_start(void);
bool cdm324_stop(void);

bool cdm324_get_latest_snapshot(
    cdm324_snapshot_t *out_snapshot);

/*
 * Copy the latest power spectrum.
 *
 * power_out must point to an array containing at least
 * CDM324_SPECTRUM_BIN_COUNT float elements.
 *
 * The returned timestamp corresponds to the spectrum.
 */
bool cdm324_get_latest_spectrum(
    float *power_out,
    size_t power_count,
    uint32_t *time_us_out);