#ifndef INC_RTC_H
#define INC_RTC_H

#include <stdint.h>

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_datetime_t;

void rtc_module_init(void);
rtc_datetime_t rtc_get_timestamp(void);

#endif //INC_RTC_H
