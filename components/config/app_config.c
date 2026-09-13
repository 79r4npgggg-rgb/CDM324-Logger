#include "app_config.h"

static app_config_t g_app_config = {
    .snapshot_period_ms = APP_SNAPSHOT_PERIOD_MS,
    .wheel_diameter_mm = APP_CDM324_WHEEL_DIAMETER_MM,
    .csv_buffer_lines = APP_CSV_BUFFER_LINES,
    .csv_line_max_len = APP_CSV_LINE_MAX_LEN,
    .log_queue_len = APP_LOG_QUEUE_LEN,
    .cdm324_queue_len = APP_CDM324_QUEUE_LEN,
    .enabled = true,
};

void app_config_init(void)
{
    /*
     * This is intentionally kept simple at the first stage.
     * GPIO mappings and other project-specific values should be centralized here
     * as the project grows, without introducing unnecessary abstraction.
     */
}

const app_config_t *app_config_get(void)
{
    return &g_app_config;
}
