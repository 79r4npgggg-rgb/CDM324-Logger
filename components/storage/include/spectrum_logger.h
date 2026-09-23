#pragma once

#include <stdbool.h>
#include <stdint.h>

bool spectrum_logger_init(void);

bool spectrum_logger_submit_latest(void);

bool spectrum_logger_set_logging_enabled(
    bool enabled);

bool spectrum_logger_is_logging_enabled(void);