#include "sdcard.h"
#include "central.h"
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"

#define SD_MOUNT_POINT      "/sdcard"
#define SD_SKID_CSV_PATH    SD_MOUNT_POINT "/resultadosSkid.csv"
#define SD_ACCEL_CSV_PATH   SD_MOUNT_POINT "/resultadosAccel.csv"

#define SD_MAX_ATTEMPTS   6 // first attempt + 5 retries
#define SD_RETRY_DELAY_MS 50

static const char *SD_TAG = "SDCARD";

static sdmmc_card_t *sd_card = NULL;
static bool sd_mounted = false;

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

    esp_err_t err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(SD_TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
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

    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);
    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Failed to mount filesystem. If the card isn't already FAT-formatted, "
                          "set format_if_mount_failed = true, or format it on a PC.");
        } else {
            ESP_LOGE(SD_TAG, "Failed to initialize SD card (%s). Check wiring: MOSI=%d, SCK=%d, MISO=%d, "
                          "and make sure CS is tied low on the module.",
                     esp_err_to_name(err), SD_MOSI_GPIO, SD_SCK_GPIO, SD_MISO_GPIO);
        }
        spi_bus_free(host.slot);
        return;
    }

    sd_mounted = true;
    ESP_LOGI(SD_TAG, "SD card mounted at %s", SD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, sd_card);
}

// Opens path, retrying up to SD_MAX_ATTEMPTS times on transient failures.
// "File doesn't exist" is not treated as a failure worth retrying: *out_not_found
// is set and NULL is returned immediately.
static FILE *fopen_with_retry(const char *path, const char *mode, bool *out_not_found) {
    if (out_not_found != NULL) {
        *out_not_found = false;
    }

    for (int attempt = 1; attempt <= SD_MAX_ATTEMPTS; attempt++) {
        errno = 0;
        FILE *f = fopen(path, mode);
        if (f != NULL) {
            return f;
        }

        if (errno == ENOENT) {
            if (out_not_found != NULL) {
                *out_not_found = true;
            }
            return NULL;
        }

        ESP_LOGE(SD_TAG, "Failed to open %s (mode '%s', attempt %d/%d): %s",
                 path, mode, attempt, SD_MAX_ATTEMPTS, strerror(errno));
        if (attempt < SD_MAX_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(SD_RETRY_DELAY_MS));
        }
    }

    ESP_LOGE(SD_TAG, "Giving up opening %s after %d attempts", path, SD_MAX_ATTEMPTS);
    return NULL;
}

static bool file_exists(const char *path) {
    FILE *f = fopen_with_retry(path, "r", NULL);
    if (f == NULL) {
        return false;
    }
    fclose(f);
    return true;
}

void sdcard_log_skid_result(rtc_datetime_t timestamp, const char *team, int driver,
                             float lap1, float lap2, float lap3, float lap4, float avg) {
    if (!sd_mounted) {
        ESP_LOGE(SD_TAG, "Cannot log skid result: SD card is not mounted");
        return;
    }

    for (int attempt = 1; attempt <= SD_MAX_ATTEMPTS; attempt++) {
        bool needs_header = !file_exists(SD_SKID_CSV_PATH);

        errno = 0;
        FILE *f = fopen(SD_SKID_CSV_PATH, "a");
        if (f == NULL) {
            ESP_LOGE(SD_TAG, "Failed to open %s for append (attempt %d/%d): %s",
                     SD_SKID_CSV_PATH, attempt, SD_MAX_ATTEMPTS, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(SD_RETRY_DELAY_MS));
            continue;
        }

        if (needs_header) {
            fprintf(f, "timestamp,carro,piloto,volta1,volta2,volta3,volta4,media\n");
        }

        fprintf(f, "%04u-%02u-%02u %02u:%02u:%02u,%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            timestamp.year, timestamp.month, timestamp.day,
            timestamp.hour, timestamp.minute, timestamp.second,
            team, driver,
            lap1, lap2, lap3, lap4, avg);

        if (fclose(f) != 0) {
            ESP_LOGE(SD_TAG, "Failed to flush/close %s (attempt %d/%d): %s",
                     SD_SKID_CSV_PATH, attempt, SD_MAX_ATTEMPTS, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(SD_RETRY_DELAY_MS));
            continue;
        }

        ESP_LOGI(SD_TAG, "Logged skid result to %s", SD_SKID_CSV_PATH);
        return;
    }

    ESP_LOGE(SD_TAG, "Giving up logging skid result to %s after %d attempts", SD_SKID_CSV_PATH, SD_MAX_ATTEMPTS);
}

void sdcard_log_accel_result(rtc_datetime_t timestamp, const char *team, int driver, float time_s) {
    if (!sd_mounted) {
        ESP_LOGE(SD_TAG, "Cannot log accel result: SD card is not mounted");
        return;
    }

    for (int attempt = 1; attempt <= SD_MAX_ATTEMPTS; attempt++) {
        bool needs_header = !file_exists(SD_ACCEL_CSV_PATH);

        errno = 0;
        FILE *f = fopen(SD_ACCEL_CSV_PATH, "a");
        if (f == NULL) {
            ESP_LOGE(SD_TAG, "Failed to open %s for append (attempt %d/%d): %s",
                     SD_ACCEL_CSV_PATH, attempt, SD_MAX_ATTEMPTS, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(SD_RETRY_DELAY_MS));
            continue;
        }

        if (needs_header) {
            fprintf(f, "timestamp,carro,piloto,tempo\n");
        }

        fprintf(f, "%04u-%02u-%02u %02u:%02u:%02u,%s,%d,%.3f\n",
            timestamp.year, timestamp.month, timestamp.day,
            timestamp.hour, timestamp.minute, timestamp.second,
            team, driver,
            time_s);

        if (fclose(f) != 0) {
            ESP_LOGE(SD_TAG, "Failed to flush/close %s (attempt %d/%d): %s",
                     SD_ACCEL_CSV_PATH, attempt, SD_MAX_ATTEMPTS, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(SD_RETRY_DELAY_MS));
            continue;
        }

        ESP_LOGI(SD_TAG, "Logged accel result to %s", SD_ACCEL_CSV_PATH);
        return;
    }

    ESP_LOGE(SD_TAG, "Giving up logging accel result to %s after %d attempts", SD_ACCEL_CSV_PATH, SD_MAX_ATTEMPTS);
}

#define SD_LAST_RESULTS_COUNT 5
#define SD_LAST_LINE_LEN      64

// Copies the (0-indexed) field_index'th comma-separated field of line into out
static bool csv_field_copy(const char *line, int field_index, char *out, size_t out_len) {
    const char *p = line;
    for (int i = 0; i < field_index; i++) {
        p = strchr(p, ',');
        if (p == NULL) {
            out[0] = '\0';
            return false;
        }
        p++;
    }

    const char *end = strchr(p, ',');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len >= out_len) {
        len = out_len - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

// Reads path (skipping the header row) and keeps the last max_lines rows, oldest first
static int read_last_lines(const char *path, char lines[][SD_LAST_LINE_LEN], int max_lines) {
    FILE *f = fopen_with_retry(path, "r", NULL);
    if (f == NULL) {
        return 0; // either no results logged yet, or failed after retries (already logged)
    }

    char buf[SD_LAST_LINE_LEN];
    if (fgets(buf, sizeof(buf), f) == NULL) { // skip header row
        fclose(f);
        return 0;
    }

    int count = 0;
    while (fgets(buf, sizeof(buf), f) != NULL) {
        size_t len = strlen(buf);
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
            buf[--len] = '\0';
        }
        if (len == 0) {
            continue;
        }

        if (count < max_lines) {
            strncpy(lines[count], buf, SD_LAST_LINE_LEN - 1);
            lines[count][SD_LAST_LINE_LEN - 1] = '\0';
            count++;
        } else {
            for (int i = 1; i < max_lines; i++) {
                strcpy(lines[i - 1], lines[i]);
            }
            strncpy(lines[max_lines - 1], buf, SD_LAST_LINE_LEN - 1);
            lines[max_lines - 1][SD_LAST_LINE_LEN - 1] = '\0';
        }
    }
    fclose(f);

    return count < max_lines ? count : max_lines;
}

void sdcard_get_last_skid_results(char *out, size_t out_len) {
    out[0] = '\0';
    if (!sd_mounted) {
        return;
    }

    char lines[SD_LAST_RESULTS_COUNT][SD_LAST_LINE_LEN];
    int count = read_last_lines(SD_SKID_CSV_PATH, lines, SD_LAST_RESULTS_COUNT);

    size_t pos = 0;
    for (int i = count - 1; i >= 0; i--) { // most recent first
        char team[8];
        char avg[16];
        csv_field_copy(lines[i], 1, team, sizeof(team));
        csv_field_copy(lines[i], 7, avg, sizeof(avg));

        int written = snprintf(out + pos, out_len - pos, "Carro #%s: %ss\n", team, avg);
        if (written < 0 || (size_t)written >= out_len - pos) {
            break;
        }
        pos += (size_t)written;
    }
}

void sdcard_get_last_accel_results(char *out, size_t out_len) {
    out[0] = '\0';
    if (!sd_mounted) {
        return;
    }

    char lines[SD_LAST_RESULTS_COUNT][SD_LAST_LINE_LEN];
    int count = read_last_lines(SD_ACCEL_CSV_PATH, lines, SD_LAST_RESULTS_COUNT);

    size_t pos = 0;
    for (int i = count - 1; i >= 0; i--) { // most recent first
        char team[8];
        char time_s[16];
        csv_field_copy(lines[i], 1, team, sizeof(team));
        csv_field_copy(lines[i], 3, time_s, sizeof(time_s));

        int written = snprintf(out + pos, out_len - pos, "Equipe %s: %ss\n", team, time_s);
        if (written < 0 || (size_t)written >= out_len - pos) {
            break;
        }
        pos += (size_t)written;
    }
}
