#ifndef INC_RTC_SET_TIME_TOOL_H
#define INC_RTC_SET_TIME_TOOL_H

// One-time utility: sets the DS1307 RTC to this firmware's compile date/time.
// Call this once, flash, check the "RTC_SET_TIME" log line to confirm it took,
// then remove the call and reflash -- the DS1307 keeps its own time afterward
// via its battery-backed oscillator.
void rtc_set_time_to_build_time(void);

#endif //INC_RTC_SET_TIME_TOOL_H
