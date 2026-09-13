#pragma once

#include <stdint.h>

#define CDM324_STATUS_NORMAL    0U
#define CDM324_STATUS_OVERFLOW  1U
#define CDM324_STATUS_INVALID   2U
#define CDM324_STATUS_NO_SIGNAL 3U

typedef struct {
    uint32_t timestamp_us;
    uint32_t status;
} cdm324_edge_event_t;
