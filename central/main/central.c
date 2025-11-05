#include "central.h"

void app_main(void) {
    waveshare_esp32_s3_rgb_lcd_init();
    if (lvgl_port_lock(-1)) {
        ui_init();
        // Release the mutex
        lvgl_port_unlock();
    }

    fsaeSkid_init();
}