#include "rtc.h"
#include "central.h"
#include "driver/i2c.h"
#include "esp_log.h"

// DS1307 RTC on the TinyRTC I2C module. This shares the same I2C bus
// (GPIO8 SDA / GPIO9 SCL, I2C_NUM_0) that waveshare_esp32_s3_rgb_lcd_init()
// already installs for the GT911 touch controller -- do not install a
// second I2C driver instance on this port, it will conflict with it.
#define DS1307_I2C_ADDR       0x68
#define DS1307_REG_SECONDS    0x00
#define DS1307_I2C_TIMEOUT_MS 100

static const char *RTC_TAG = "RTC";

static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t dec) {
    return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

rtc_datetime_t rtc_get_timestamp(void) {
    rtc_datetime_t dt = {0};

    uint8_t reg = DS1307_REG_SECONDS;
    uint8_t raw[7] = {0};

    esp_err_t err = i2c_master_write_read_device(I2C_NUM_0, DS1307_I2C_ADDR, &reg, 1,
                                                  raw, sizeof(raw), pdMS_TO_TICKS(DS1307_I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(RTC_TAG, "Failed to read DS1307 at 0x%02X on I2C_NUM_0: %s", DS1307_I2C_ADDR, esp_err_to_name(err));
        return dt;
    }

    if (raw[0] & 0x80) {
        ESP_LOGW(RTC_TAG, "DS1307 clock-halt bit is set: the oscillator isn't running (dead/missing battery, or never set)");
    }

    // DS1307 registers are BCD; assumes the RTC is kept in 24-hour mode
    dt.second = bcd_to_dec(raw[0] & 0x7F); // bit 7 is the clock-halt flag
    dt.minute = bcd_to_dec(raw[1] & 0x7F);
    dt.hour   = bcd_to_dec(raw[2] & 0x3F); // bit 6 selects 12/24-hour mode
    dt.day    = bcd_to_dec(raw[4] & 0x3F);
    dt.month  = bcd_to_dec(raw[5] & 0x1F);
    dt.year   = 2000 + bcd_to_dec(raw[6]);

    ESP_LOGI(RTC_TAG, "Timestamp: %04u-%02u-%02u %02u:%02u:%02u",
             dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);

    return dt;
}

void rtc_set_time(const rtc_datetime_t *dt) {
    uint8_t raw[8];
    raw[0] = DS1307_REG_SECONDS;
    raw[1] = dec_to_bcd(dt->second); // bit 7 (clock-halt) left cleared, so the oscillator runs
    raw[2] = dec_to_bcd(dt->minute);
    raw[3] = dec_to_bcd(dt->hour);   // 24-hour mode (bit 6 left cleared)
    raw[4] = 1;                       // day-of-week register; unused elsewhere, any valid 1-7 works
    raw[5] = dec_to_bcd(dt->day);
    raw[6] = dec_to_bcd(dt->month);
    raw[7] = dec_to_bcd((uint8_t)(dt->year - 2000));

    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, DS1307_I2C_ADDR, raw, sizeof(raw),
                                                pdMS_TO_TICKS(DS1307_I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(RTC_TAG, "Failed to write DS1307 at 0x%02X on I2C_NUM_0: %s", DS1307_I2C_ADDR, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(RTC_TAG, "RTC time set to %04u-%02u-%02u %02u:%02u:%02u",
             dt->year, dt->month, dt->day, dt->hour, dt->minute, dt->second);
}
