#include "sdcard.h"
#include "central.h"
#include <stdio.h>
#include <stdbool.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

#define SD_MOUNT_POINT      "/sdcard"
#define SD_SKID_CSV_PATH    SD_MOUNT_POINT "/resultadosSkid.csv"
#define SD_ACCEL_CSV_PATH   SD_MOUNT_POINT "/resultadosAccel.csv"

static sdmmc_card_t *sd_card = NULL;

void sdcard_init(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    if (spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA) != ESP_OK) {
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = host.slot;
    slot_config.gpio_cs = GPIO_NUM_NC; // CS is hardwired on the module; only device on the bus

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);
}

static bool file_exists(const char *path) {
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return false;
    }
    fclose(f);
    return true;
}

void sdcard_log_skid_result(rtc_datetime_t timestamp, float lap1, float lap2, float lap3, float lap4, float avg) {
    bool needs_header = !file_exists(SD_SKID_CSV_PATH);

    FILE *f = fopen(SD_SKID_CSV_PATH, "a");
    if (f == NULL) {
        return;
    }

    if (needs_header) {
        fprintf(f, "timestamp,carro,piloto,volta1,volta2,volta3,volta4,media\n");
    }

    fprintf(f, "%04u-%02u-%02u %02u:%02u:%02u,,,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        timestamp.year, timestamp.month, timestamp.day,
        timestamp.hour, timestamp.minute, timestamp.second,
        lap1, lap2, lap3, lap4, avg);

    fclose(f);
}

void sdcard_log_accel_result(rtc_datetime_t timestamp, float time_s) {
    bool needs_header = !file_exists(SD_ACCEL_CSV_PATH);

    FILE *f = fopen(SD_ACCEL_CSV_PATH, "a");
    if (f == NULL) {
        return;
    }

    if (needs_header) {
        fprintf(f, "timestamp,carro,piloto,tempo\n");
    }

    fprintf(f, "%04u-%02u-%02u %02u:%02u:%02u,,,%.3f\n",
        timestamp.year, timestamp.month, timestamp.day,
        timestamp.hour, timestamp.minute, timestamp.second,
        time_s);

    fclose(f);
}
