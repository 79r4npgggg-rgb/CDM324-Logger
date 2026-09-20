#include "board.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "board";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_oled_dev = NULL;
static bool s_oled_probe_attempted = false;
static uint8_t s_oled_buffer[128 * 8];
static uint8_t s_oled_i2c_addr = BOARD_OLED_I2C_ADDR;

static const uint8_t s_font[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x4F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x72,0x49,0x49,0x49,0x36}, {0x18,0x14,0x12,0x7F,0x10},
    {0x62,0x51,0x49,0x49,0x46}, {0x22,0x41,0x49,0x49,0x3E}, {0x7E,0x09,0x09,0x09,0x00}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x09,0x19,0x29,0x46}, {0x26,0x49,0x49,0x49,0x32}, {0x03,0x01,0x7F,0x01,0x03},
    {0x7F,0x40,0x40,0x40,0x7F}, {0x00,0x7F,0x00,0x7F,0x00}, {0x36,0x49,0x49,0x49,0x7F}, {0x41,0x7F,0x41,0x00,0x00},
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x00,0x7F,0x40,0x40,0x00},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x7F}, {0x7F,0x02,0x04,0x08,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x09,0x19,0x29,0x46},
    {0x26,0x49,0x49,0x49,0x32}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x7F,0x20,0x18,0x20,0x7F}, {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43},
    {0x00,0x7F,0x41,0x41,0x00}, {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40}, {0x00,0x03,0x07,0x08,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x42,0x00},
    {0x20,0x54,0x54,0x54,0x3C}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02},
    {0x00,0x46,0x49,0x49,0x31}, {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x01}, {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x19,0x01}, {0x3E,0x41,0x49,0x49,0x7A}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F}, {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
    {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00}, {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40}, {0x00,0x03,0x07,0x08,0x00}, {0x00,0x00,0x00,0x00,0x00}
};

static void oled_write_cmd(uint8_t cmd)
{
    if (s_oled_dev == NULL) {
        return;
    }

    uint8_t data[2] = {0x00, cmd};
    esp_err_t ret = i2c_master_transmit(s_oled_dev, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED command write failed: %s", esp_err_to_name(ret));
        s_oled_dev = NULL;
    }
}

static void oled_write_buf(const uint8_t *data, size_t len)
{
    if (s_oled_dev == NULL) {
        return;
    }

    uint8_t tx[1 + 128];
    if (len > 128) {
        len = 128;
    }
    tx[0] = 0x40;
    memcpy(&tx[1], data, len);
    esp_err_t ret = i2c_master_transmit(s_oled_dev, tx, len + 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED buffer write failed: %s", esp_err_to_name(ret));
        s_oled_dev = NULL;
    }
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_write_cmd(0xB0 | (page & 0x07));
    oled_write_cmd(col & 0x0F);
    oled_write_cmd(0x10 | ((col >> 4) & 0x0F));
}

static void oled_set_pixel(int x, int y)
{
    if (x < 0 || x >= 128 || y < 0 || y >= 64) {
        return;
    }
    const int page = y / 8;
    const int bit = y % 8;
    s_oled_buffer[page * 128 + x] |= (1U << bit);
}

static void oled_draw_char(int x, int y_page, char ch)
{
    if (ch < 0x20 || ch > 0x7E) {
        ch = ' ';
    }
    const uint8_t *glyph = s_font[(uint8_t)(ch - 0x20)];
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            if ((glyph[col] >> row) & 0x01U) {
                oled_set_pixel(x + col, y_page * 8 + row);
            }
        }
    }
}

static void oled_draw_text(int x, int y_page, const char *text)
{
    if (text == NULL) {
        return;
    }

    for (size_t i = 0; text[i] != '\0'; ++i) {
        oled_draw_char(x + (int)i * 6, y_page, text[i]);
    }
}

bool board_oled_probe(void)
{
    if (s_oled_dev != NULL) {
        return true;
    }

    if (s_oled_probe_attempted) {
        return false;
    }

    s_oled_probe_attempted = true;

    if (s_i2c_bus != NULL) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }

    board_oled_init();
    return s_oled_dev != NULL;
}

void board_oled_init(void)
{
    if (s_oled_dev != NULL) {
        return;
    }

    gpio_set_pull_mode(BOARD_I2C_SDA_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BOARD_I2C_SCL_GPIO, GPIO_PULLUP_ONLY);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 64,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        s_i2c_bus = NULL;
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BOARD_OLED_I2C_ADDR,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    ret = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_oled_dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED add device 0x%02X failed: %s", BOARD_OLED_I2C_ADDR, esp_err_to_name(ret));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        s_oled_dev = NULL;
        return;
    }

    s_oled_i2c_addr = BOARD_OLED_I2C_ADDR;
    s_oled_probe_attempted = true;
    ESP_LOGI(TAG, "OLED device registered at 0x%02X", s_oled_i2c_addr);

    static const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF
    };

    uint8_t init_packet[1 + sizeof(init_cmds)];
    init_packet[0] = 0x00;
    memcpy(&init_packet[1], init_cmds, sizeof(init_cmds));

    ret = i2c_master_transmit(s_oled_dev, init_packet, sizeof(init_packet), 1000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED init sequence NACK/timeout: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(s_oled_dev);
        i2c_del_master_bus(s_i2c_bus);
        s_oled_dev = NULL;
        s_i2c_bus = NULL;
        s_oled_probe_attempted = true;
        return;
    }

    board_oled_clear();
    ESP_LOGI(TAG, "OLED initialized");
}

void board_oled_clear(void)
{
    if (s_oled_dev == NULL) {
        return;
    }

    memset(s_oled_buffer, 0, sizeof(s_oled_buffer));
    for (int page = 0; page < 8; ++page) {
        oled_set_cursor(page, 0);
        oled_write_buf(&s_oled_buffer[page * 128], 128);
    }
}

void board_oled_write_text(const char *text)
{
    if (s_oled_dev == NULL || text == NULL) {
        return;
    }

    memset(s_oled_buffer, 0, sizeof(s_oled_buffer));
    oled_draw_text(0, 0, text);

    for (int page = 0; page < 8; ++page) {
        oled_set_cursor(page, 0);
        oled_write_buf(&s_oled_buffer[page * 128], 128);
    }
}

void board_oled_show_status(int32_t raw_hz, int32_t velocity_mmps, bool logging_active)
{
    if (s_oled_dev == NULL) {
        return;
    }

    memset(s_oled_buffer, 0, sizeof(s_oled_buffer));

    char line1[24];
    char line2[24];
    char line3[24];
    snprintf(line1, sizeof(line1), "RAW:%ld", (long)raw_hz);
    snprintf(line2, sizeof(line2), "SPD:%ld", (long)velocity_mmps);
    snprintf(line3, sizeof(line3), "SD:%s", logging_active ? "ON" : "OFF");

    oled_draw_text(0, 0, line1);
    oled_draw_text(0, 1, line2);
    oled_draw_text(0, 2, line3);

    for (int page = 0; page < 8; ++page) {
        oled_set_cursor(page, 0);
        oled_write_buf(&s_oled_buffer[page * 128], 128);
    }
}

void board_i2c_scan(void)
{
    i2c_master_bus_handle_t bus = s_i2c_bus;
    bool own_bus = false;

    if (bus == NULL) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = BOARD_I2C_SDA_GPIO,
            .scl_io_num = BOARD_I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 20,
            .flags = {
                .enable_internal_pullup = true,
                .allow_pd = false,
            },
        };

        esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2C scan failed to initialize bus: %s", esp_err_to_name(ret));
            return;
        }
        own_bus = true;
    }

    int found = 0;
    for (uint8_t addr = 0; addr < 0x80; ++addr) {
        esp_err_t ret = i2c_master_probe(bus, addr, 50);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "No I2C device responded on the bus.");
    } else {
        ESP_LOGI(TAG, "I2C scan complete: %d device(s) responded.", found);
    }

    if (own_bus && bus != NULL) {
        i2c_del_master_bus(bus);
    }
}

void board_diagnostics_run(void)
{
    ESP_LOGI(TAG, "GPIO map review:");
    ESP_LOGI(TAG, "  CDM324  : GPIO%d", BOARD_GPIO_CDM324_FOUT);
    ESP_LOGI(TAG, "  Button  : GPIO%d", BOARD_GPIO_LOG_BUTTON);
    ESP_LOGI(TAG, "  LED     : GPIO%d", BOARD_GPIO_LED);
    ESP_LOGI(TAG, "  OLED I2C: SDA=%d SCL=%d ADDR=0x%02X", BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO, BOARD_OLED_I2C_ADDR);
    ESP_LOGI(TAG, "  SD SPI  : CS=%d SCK=%d MISO=%d MOSI=%d", BOARD_SD_SPI_CS, BOARD_SD_SPI_SCLK, BOARD_SD_SPI_MISO, BOARD_SD_SPI_MOSI);

    board_i2c_scan();

    if (board_oled_probe()) {
        ESP_LOGI(TAG, "OLED diagnostic: recognized on I2C bus");
    } else {
        ESP_LOGW(TAG, "OLED diagnostic: not recognized on I2C bus");
    }
}

void board_init(void)
{
    gpio_config_t input_conf = {
    .pin_bit_mask = (1ULL << BOARD_GPIO_CDM324_FOUT) |
                    (1ULL << BOARD_GPIO_LOG_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << BOARD_GPIO_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_config(&output_conf));

    gpio_set_level(BOARD_GPIO_LED, 0);
    gpio_set_pull_mode(BOARD_GPIO_LOG_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_direction(BOARD_GPIO_LOG_BUTTON, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "Board initialized");
}
