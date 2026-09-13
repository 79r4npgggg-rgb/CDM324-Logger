#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CSV_LOG_HEADER "time_us,level,freq_hz,velocity_mmps,status\n"

typedef struct {
    uint32_t time_us;
    int32_t level;
    int32_t freq_hz;
    int32_t velocity_mmps;
    uint32_t status;
} csv_snapshot_t;

bool csv_logger_init(void);
bool csv_logger_queue_snapshot(const csv_snapshot_t *snapshot);
bool csv_logger_flush_pending(void);
bool csv_logger_set_logging_enabled(bool enabled);
bool csv_logger_is_logging_enabled(void);
void csv_logger_set_overflow(uint32_t overflow_count, uint32_t dropped_samples);
