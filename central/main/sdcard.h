#ifndef INC_SDCARD_H
#define INC_SDCARD_H

#include "rtc.h"

void sdcard_init(void);
void sdcard_log_skid_result(rtc_datetime_t timestamp, float lap1, float lap2, float lap3, float lap4, float avg);
void sdcard_log_accel_result(rtc_datetime_t timestamp, float time_s);

#endif //INC_SDCARD_H
