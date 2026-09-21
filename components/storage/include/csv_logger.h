#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CSV_LOG_HEADER \
    "time_us,aout_dc_mv,aout_rms_mv,aout_pp_mv,doppler_hz,status\n"

typedef struct {
    uint32_t time_us;

    int32_t aout_dc_mv;
    int32_t aout_rms_mv;
    int32_t aout_pp_mv;

    int32_t doppler_hz;

    uint32_t status;
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