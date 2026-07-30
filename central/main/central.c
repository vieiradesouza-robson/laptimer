#include "central.h"
#include "rtc_set_time_tool.h"

void app_main(void) {
    sdcard_init();

    waveshare_esp32_s3_rgb_lcd_init();
    if (lvgl_port_lock(-1)) {
        ui_init();
        // Release the mutex
        lvgl_port_unlock();
    }

    // One-time RTC clock set: uncomment, flash, check the "RTC_SET_TIME" log
    // line to confirm it took, then comment this back out and reflash.
    // rtc_set_time_to_build_time();
}