#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CSV_MAX_PEAKS 5U

#define CSV_LOG_HEADER \
    "time_us,aout_dc_mv,aout_rms_mv,aout_pp_mv,doppler_hz,status,peak_count,peak1_hz,peak1_power,peak2_hz,peak2_power,peak3_hz,peak3_power,peak4_hz,peak4_power,peak5_hz,peak5_power\n"

typedef struct {
    uint32_t time_us;

    int32_t aout_dc_mv;
    int32_t aout_rms_mv;
    int32_t aout_pp_mv;

    int32_t doppler_hz;

    uint32_t status;

    uint32_t peak_count;

    struct {
        int32_t frequency_hz;
        float power;
    } peaks[CSV_MAX_PEAKS];

} csv_snapshot_t;

bool csv_logger_init(void);

bool csv_logger_queue_snapshot(
    const csv_snapshot_t *snapshot);

bool csv_logger_set_logging_enabled(
    bool enabled);

bool csv_logger_is_logging_enabled(void);

void csv_logger_set_overflow(
    uint32_t overflow_count,
    uint32_t dropped_samples);