#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define CDM324_EVENT_QUEUE_LENGTH 16
#define CDM324_MAX_LEVEL 2
#define CDM324_MIN_LEVEL 0

#define CDM324_STATUS_NORMAL    0U
#define CDM324_STATUS_OVERFLOW  1U
#define CDM324_STATUS_INVALID   2U
#define CDM324_STATUS_NO_SIGNAL 3U

typedef struct {
    uint32_t time_us;
    uint32_t status;
    int32_t level;
    int32_t freq_hz;
    int32_t velocity_mmps;
    int32_t vout_mv;
} cdm324_snapshot_t;

int32_t cdm324_velocity_mmps_from_freq_hz(int32_t freq_hz, uint32_t wheel_diameter_mm);
void cdm324_gpio_isr_handler(void *arg);
void cdm324_set_event_queue(QueueHandle_t queue);
bool cdm324_init(void);
bool cdm324_start(void);
bool cdm324_stop(void);
bool cdm324_get_latest_snapshot(cdm324_snapshot_t *out_snapshot);
QueueHandle_t cdm324_get_event_queue(void);
