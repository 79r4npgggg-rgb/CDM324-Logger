#include "cdm324.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "cdm324_types.h"

static QueueHandle_t s_event_queue = NULL;

void cdm324_set_event_queue(QueueHandle_t queue)
{
    s_event_queue = queue;
}

void IRAM_ATTR cdm324_gpio_isr_handler(void *arg)
{
    (void)arg;
    uint32_t now_us = esp_timer_get_time();
    cdm324_edge_event_t event = {
        .timestamp_us = now_us,
        .status = 0U,
    };

    if (s_event_queue != NULL) {
        BaseType_t hp_task_woken = pdFALSE;
        xQueueSendFromISR(s_event_queue, &event, &hp_task_woken);
        if (hp_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}
