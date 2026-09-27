#include "input_diag.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "board.h"

static const char *TAG = "INPUT_DIAG";

/*
 * Edge counters.
 *
 * These are incremented from GPIO ISR context.
 */
static volatile uint32_t s_qre_edges = 0;
static volatile uint32_t s_esc_edges = 0;

static portMUX_TYPE s_diag_spinlock =
    portMUX_INITIALIZER_UNLOCKED;


/*
 * GPIO ISR
 *
 * Keep this extremely small.
 */
static void IRAM_ATTR input_diag_gpio_isr(void *arg)
{
    const uint32_t gpio_num =
        (uint32_t)(uintptr_t)arg;

    if (gpio_num == BOARD_GPIO_QRE_INPUT) {

        s_qre_edges++;

    } else if (gpio_num == BOARD_GPIO_ESC_PWM) {

        s_esc_edges++;
    }
}


bool input_diag_init(void)
{
    ESP_LOGI(
        TAG,
        "Initializing GPIO edge diagnostics: QRE=GPIO%d ESC=GPIO%d",
        BOARD_GPIO_QRE_INPUT,
        BOARD_GPIO_ESC_PWM
    );

    /*
     * IMPORTANT:
     *
     * Do not call gpio_config() here.
     *
     * QRE and ESC components have already configured
     * their respective GPIOs for PCNT/RMT.
     *
     * We only enable GPIO edge interrupts on the
     * existing pin configuration.
     */

    esp_err_t err;

    /*
     * Install GPIO ISR service.
     *
     * It may already be installed by another component,
     * so ESP_ERR_INVALID_STATE is treated as harmless.
     */
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "gpio_install_isr_service failed: %s",
            esp_err_to_name(err)
        );

        return false;
    }

    /*
     * QRE GPIO: count both rising and falling edges.
     */
    err = gpio_set_intr_type(
        BOARD_GPIO_QRE_INPUT,
        GPIO_INTR_ANYEDGE
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to configure QRE GPIO interrupt: %s",
            esp_err_to_name(err)
        );

        return false;
    }

    /*
     * ESC PWM GPIO: count both rising and falling edges.
     */
    err = gpio_set_intr_type(
        BOARD_GPIO_ESC_PWM,
        GPIO_INTR_ANYEDGE
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to configure ESC GPIO interrupt: %s",
            esp_err_to_name(err)
        );

        return false;
    }

    /*
     * Register handlers.
     *
     * Pass GPIO number as ISR argument.
     */
    err = gpio_isr_handler_add(
        BOARD_GPIO_QRE_INPUT,
        input_diag_gpio_isr,
        (void *)(uintptr_t)BOARD_GPIO_QRE_INPUT
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to add QRE GPIO ISR: %s",
            esp_err_to_name(err)
        );

        return false;
    }

    err = gpio_isr_handler_add(
        BOARD_GPIO_ESC_PWM,
        input_diag_gpio_isr,
        (void *)(uintptr_t)BOARD_GPIO_ESC_PWM
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to add ESC GPIO ISR: %s",
            esp_err_to_name(err)
        );

        return false;
    }

    ESP_LOGI(
        TAG,
        "GPIO edge diagnostics ready"
    );

    return true;
}


bool input_diag_get_counts(
    input_diag_counts_t *counts,
    bool clear_after_read)
{
    if (counts == NULL) {
        return false;
    }

    /*
     * Copy counters atomically with respect to the ISR.
     */
    portENTER_CRITICAL(&s_diag_spinlock);

    counts->qre_edges = s_qre_edges;
    counts->esc_edges = s_esc_edges;

    if (clear_after_read) {
        s_qre_edges = 0;
        s_esc_edges = 0;
    }

    portEXIT_CRITICAL(&s_diag_spinlock);

    return true;
}