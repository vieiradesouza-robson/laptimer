#ifndef INC_SDCARD_H
#define INC_SDCARD_H

#include "rtc.h"
#include <stddef.h>

void sdcard_init(void);
void sdcard_log_skid_result(rtc_datetime_t timestamp, const char *team, int driver,
                             float lap1, float lap2, float lap3, float lap4, float avg);
void sdcard_log_accel_result(rtc_datetime_t timestamp, const char *team, int driver, float time_s);

// Formats up to the last 5 results (most recent first), one "Equipe X: Ys" per line, into out
void sdcard_get_last_skid_results(char *out, size_t out_len);
void sdcard_get_last_accel_results(char *out, size_t out_len);

#endif //INC_SDCARD_H
