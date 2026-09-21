#include "board.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "u8g2.h"
#include "u8x8.h"
#include "esp32_hw_i2c.h"

static const char *TAG = "board";

/* --------------------------------------------------------------------------
 * OLED / U8g2
 * -------------------------------------------------------------------------- */

static u8g2_t s_u8g2;
static bool s_oled_initialized = false;

static u8g2_esp32_i2c_ctx_t s_u8g2_i2c_ctx = {
    .cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin = BOARD_I2C_SDA_GPIO,
        .scl_pin = BOARD_I2C_SCL_GPIO,
        .clk_hz = 400000,
        .dev_addr_7bit = BOARD_OLED_I2C_ADDR,
        .timeout_ms = 1000,
        .reset_pin = U8G2_ESP32_PIN_UNUSED,
    },
};

/* --------------------------------------------------------------------------
 * OLED helper
 * -------------------------------------------------------------------------- */

static void oled_send_buffer(void)
{
    u8g2_SendBuffer(&s_u8g2);
}

/* --------------------------------------------------------------------------
 * OLED initialization
 * -------------------------------------------------------------------------- */

void board_oled_init(void)
{
    if (s_oled_initialized) {
        return;
    }

    ESP_LOGI(
        TAG,
        "Initializing OLED with U8g2: SDA=%d SCL=%d ADDR=0x%02X",
        BOARD_I2C_SDA_GPIO,
        BOARD_I2C_SCL_GPIO,
        BOARD_OLED_I2C_ADDR
    );

    /*
     * Register the ESP32 hardware-I2C context used by
     * the Nixy4 U8g2 ESP-IDF port.
     */
    esp_err_t ret = u8g2_esp32_i2c_set_default_context(
        &s_u8g2_i2c_ctx
    );

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "u8g2 I2C context setup failed: %s",
            esp_err_to_name(ret)
        );
        return;
    }

    /*
     * SSD1306 128x64 / I2C / full framebuffer.
     */
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &s_u8g2,
        U8G2_R0,
        u8x8_byte_esp32_hw_i2c,
        u8x8_gpio_and_delay_esp32_i2c
    );

    /*
     * Initialize SSD1306.
     */
    u8g2_InitDisplay(&s_u8g2);

    /*
     * Wake display from power-save mode.
     */
    u8g2_SetPowerSave(&s_u8g2, 0);

    /*
     * Text rendering settings.
     */
    u8g2_SetFont(&s_u8g2, u8g2_font_6x10_tf);
    u8g2_SetFontPosTop(&s_u8g2);
    u8g2_SetDrawColor(&s_u8g2, 1);

    /*
     * Clear display.
     */
    u8g2_ClearBuffer(&s_u8g2);
    oled_send_buffer();

    s_oled_initialized = true;

    ESP_LOGI(TAG, "OLED initialized");
}

/* --------------------------------------------------------------------------
 * OLED probe
 *
 * IMPORTANT:
 * This function intentionally does NOT draw anything.
 *
 * The old diagnostic version displayed "U8g2 OK" here, which caused the
 * production status display and the diagnostic text to overwrite each other.
 * -------------------------------------------------------------------------- */

bool board_oled_probe(void)
{
    if (!s_oled_initialized) {
        return false;
    }

    return true;
}

/* --------------------------------------------------------------------------
 * OLED clear
 * -------------------------------------------------------------------------- */

void board_oled_clear(void)
{
    if (!s_oled_initialized) {
        return;
    }

    u8g2_ClearBuffer(&s_u8g2);
    oled_send_buffer();
}

/* --------------------------------------------------------------------------
 * OLED text
 *
 * Used for simple messages such as logging state changes.
 * -------------------------------------------------------------------------- */

void board_oled_write_text(const char *text)
{
    if (!s_oled_initialized || text == NULL) {
        return;
    }

    u8g2_ClearBuffer(&s_u8g2);

    u8g2_SetFont(
        &s_u8g2,
        u8g2_font_6x10_tf
    );

    u8g2_SetFontPosTop(&s_u8g2);

    u8g2_DrawStr(
        &s_u8g2,
        0,
        0,
        text
    );

    oled_send_buffer();
}

/* --------------------------------------------------------------------------
 * OLED status display
 *
 * Display layout:
 *
 *   CDM324 LOGGER
 *   F:  20 Hz
 *   DC:2575 mV
 *   LOG: ON
 *
 * The second argument is kept as velocity_mmps in the public API for
 * compatibility, but the current application uses this field for Aout DC.
 * -------------------------------------------------------------------------- */

void board_oled_show_status(
    int32_t raw_hz,
    int32_t velocity_mmps,
    bool logging_active
)
{
    if (!s_oled_initialized) {
        return;
    }

    char line[32];

    u8g2_ClearBuffer(&s_u8g2);

    /*
     * Line 1: title
     */
    u8g2_SetFont(
        &s_u8g2,
        u8g2_font_6x10_tf
    );

    u8g2_SetFontPosTop(&s_u8g2);

    u8g2_DrawStr(
        &s_u8g2,
        0,
        0,
        "CDM324 LOGGER"
    );

    /*
     * Line 2: Doppler frequency
     */
    snprintf(
        line,
        sizeof(line),
        "F:%4ld Hz",
        (long)raw_hz
    );

    u8g2_DrawStr(
        &s_u8g2,
        0,
        16,
        line
    );

    /*
     * Line 3: Aout DC level
     *
     * NOTE:
     * The existing public argument name is velocity_mmps.
     * In the current application this value is the Aout DC value.
     */
    snprintf(
        line,
        sizeof(line),
        "DC:%4ld mV",
        (long)velocity_mmps
    );

    u8g2_DrawStr(
        &s_u8g2,
        0,
        32,
        line
    );

    /*
     * Line 4: logging state
     */
    snprintf(
        line,
        sizeof(line),
        "LOG: %s",
        logging_active ? "ON" : "OFF"
    );

    u8g2_DrawStr(
        &s_u8g2,
        0,
        48,
        line
    );

    oled_send_buffer();
}

/* --------------------------------------------------------------------------
 * I2C scan
 *
 * Diagnostic function only.
 * -------------------------------------------------------------------------- */

void board_i2c_scan(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 4,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    i2c_master_bus_handle_t bus_handle = NULL;

    esp_err_t ret = i2c_new_master_bus(
        &bus_config,
        &bus_handle
    );

    if (ret != ESP_OK) {
        ESP_LOGW(
            TAG,
            "I2C scan bus init failed: %s",
            esp_err_to_name(ret)
        );
        return;
    }

    ESP_LOGI(TAG, "I2C scan started");

    int found = 0;

    for (uint8_t addr = 1; addr < 127; ++addr) {
        ret = i2c_master_probe(
            bus_handle,
            addr,
            100
        );

        if (ret == ESP_OK) {
            ESP_LOGI(
                TAG,
                "I2C device found: 0x%02X",
                addr
            );
            found++;
        }
    }

    ESP_LOGI(
        TAG,
        "I2C scan finished: %d device(s)",
        found
    );

    i2c_del_master_bus(bus_handle);
}

/* --------------------------------------------------------------------------
 * Board diagnostics
 * -------------------------------------------------------------------------- */

void board_diagnostics_run(void)
{
    ESP_LOGI(TAG, "Board diagnostics");

    ESP_LOGI(
        TAG,
        "CDM324 Aout : GPIO%d",
        BOARD_GPIO_CDM324_AOUT
    );

    ESP_LOGI(
        TAG,
        "Button      : GPIO%d",
        BOARD_GPIO_LOG_BUTTON
    );

    ESP_LOGI(
        TAG,
        "LED         : GPIO%d",
        BOARD_GPIO_LED
    );

    ESP_LOGI(
        TAG,
        "OLED I2C    : SDA=%d SCL=%d ADDR=0x%02X",
        BOARD_I2C_SDA_GPIO,
        BOARD_I2C_SCL_GPIO,
        BOARD_OLED_I2C_ADDR
    );

    ESP_LOGI(
        TAG,
        "SD SPI      : CS=%d SCK=%d MISO=%d MOSI=%d",
        BOARD_SD_SPI_CS,
        BOARD_SD_SPI_SCLK,
        BOARD_SD_SPI_MISO,
        BOARD_SD_SPI_MOSI
    );

    board_i2c_scan();

    if (board_oled_probe()) {
        ESP_LOGI(TAG, "OLED diagnostic: OK");
    } else {
        ESP_LOGW(TAG, "OLED diagnostic: NOT READY");
    }
}

/* --------------------------------------------------------------------------
 * Board initialization
 * -------------------------------------------------------------------------- */

void board_init(void)
{
    /*
     * User button
     * XIAO Expansion Board D1 = GPIO2
     */
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_LOG_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(
        gpio_config(&button_config)
    );

    /*
     * Status LED
     * XIAO ESP32-S3 = GPIO21
     */
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(
        gpio_config(&led_config)
    );

    gpio_set_level(
        BOARD_GPIO_LED,
        0
    );

    ESP_LOGI(TAG, "Board initialized");
}