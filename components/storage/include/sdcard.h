#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define SDCARD_LOG_FILENAME "/sdcard/cdm324_log.csv"

bool sdcard_init(void);
bool sdcard_probe(void);
bool sdcard_write_csv_line(const char *line);
bool sdcard_write_csv_lines(const char *const *lines, size_t line_count);
uint32_t sdcard_get_write_count(void);
