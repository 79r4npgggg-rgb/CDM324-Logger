#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cdm324.h"

#define SPECTRUM_BIN_VERSION  1U

typedef struct {
    uint32_t time_us;

    float power[CDM324_SPECTRUM_BIN_COUNT];

} spectrum_record_t;

bool spectrum_logger_init(void);

bool spectrum_logger_queue_record(
    const spectrum_record_t *record);

bool spectrum_logger_set_logging_enabled(
    bool enabled);

bool spectrum_logger_is_logging_enabled(void);