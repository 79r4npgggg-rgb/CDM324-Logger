#pragma once

#include <stdint.h>
#include <stdbool.h>

#define APP_SNAPSHOT_PERIOD_MS      10U
#define APP_CDM324_WHEEL_DIAMETER_MM 60U
#define APP_CSV_BUFFER_LINES        32U
#define APP_CSV_LINE_MAX_LEN        512U
#define APP_LOG_QUEUE_LEN           32U
#define APP_CDM324_QUEUE_LEN        32U
#define APP_MAX_STATUS              4U

#define APP_CDM324_LEVEL_NORMAL     1
#define APP_CDM324_LEVEL_HIGH       2
#define APP_CDM324_LEVEL_LOW        0

#define APP_CDM324_STATUS_NORMAL    0U
#define APP_CDM324_STATUS_OVERFLOW  1U
#define APP_CDM324_STATUS_INVALID   2U
#define APP_CDM324_STATUS_NO_SIGNAL 3U

typedef struct {
    uint32_t snapshot_period_ms;
    uint32_t wheel_diameter_mm;
    uint32_t csv_buffer_lines;
    uint32_t csv_line_max_len;
    uint32_t log_queue_len;
    uint32_t cdm324_queue_len;
    bool enabled;
} app_config_t;

void app_config_init(void);
const app_config_t *app_config_get(void);
