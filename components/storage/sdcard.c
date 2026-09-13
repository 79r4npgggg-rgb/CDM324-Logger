#include "sdcard.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "sdmmc_cmd.h"

#include "board.h"

static const char *TAG = "sdcard";
static uint32_t s_write_count = 0;
static bool s_sd_initialized = false;

bool sdcard_probe(void)
{
    return sdcard_init();
}

bool sdcard_init(void)
{
    if (s_sd_initialized) {
        return true;
    }

    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BOARD_SD_SPI_CS;
    slot_config.host_id = host.slot;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BOARD_SD_SPI_MOSI,
        .miso_io_num = BOARD_SD_SPI_MISO,
        .sclk_io_num = BOARD_SD_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card = NULL;
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return false;
    }

    s_sd_initialized = true;
    ESP_LOGI(TAG, "SD card mounted");
    return true;
}

bool sdcard_write_csv_line(const char *line)
{
    if (line == NULL) {
        return false;
    }

    FILE *fp = fopen(SDCARD_LOG_FILENAME, "a");
    if (fp == NULL) {
        ESP_LOGE(TAG, "Unable to open %s for append", SDCARD_LOG_FILENAME);
        return false;
    }

    if (fputs(line, fp) == EOF) {
        fclose(fp);
        ESP_LOGE(TAG, "Failed to write CSV line");
        return false;
    }

    fflush(fp);
    fclose(fp);
    s_write_count++;
    return true;
}

bool sdcard_write_csv_lines(const char *const *lines, size_t line_count)
{
    if (lines == NULL || line_count == 0) {
        return false;
    }

    FILE *fp = fopen(SDCARD_LOG_FILENAME, "a");
    if (fp == NULL) {
        ESP_LOGE(TAG, "Unable to open %s for append", SDCARD_LOG_FILENAME);
        return false;
    }

    for (size_t i = 0; i < line_count; ++i) {
        if (lines[i] == NULL) {
            continue;
        }

        if (fputs(lines[i], fp) == EOF) {
            fclose(fp);
            ESP_LOGE(TAG, "Failed to write one CSV line");
            return false;
        }
    }

    fflush(fp);
    fclose(fp);
    s_write_count += line_count;
    return true;
}

uint32_t sdcard_get_write_count(void)
{
    return s_write_count;
}
