#include "rtc_set_time_tool.h"
#include "rtc.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static const char *RTC_SET_TOOL_TAG = "RTC_SET_TIME";

static uint8_t month_from_name(const char *name) {
    static const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    for (uint8_t i = 0; i < 12; i++) {
        if (strncmp(name, months[i], 3) == 0) {
            return i + 1;
        }
    }
    return 1; // should never happen with a valid __DATE__
}

void rtc_set_time_to_build_time(void) {
    // __DATE__ is "Mmm dd yyyy" (e.g. "Jul 30 2026"), __TIME__ is "hh:mm:ss"
    char mon_name[4] = {0};
    int day = 0, year = 0;
    int hour = 0, minute = 0, second = 0;

    sscanf(__DATE__, "%3s %d %d", mon_name, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

    rtc_datetime_t dt = {
        .year = (uint16_t)year,
        .month = month_from_name(mon_name),
        .day = (uint8_t)day,
        .hour = (uint8_t)hour,
        .minute = (uint8_t)minute,
        .second = (uint8_t)second,
    };

    ESP_LOGI(RTC_SET_TOOL_TAG, "Setting RTC to build time: %04u-%02u-%02u %02u:%02u:%02u",
             dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);

    rtc_set_time(&dt);
}
