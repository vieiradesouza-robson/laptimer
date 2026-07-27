#ifndef INC_CENTRAL_H
#define INC_CENTRAL_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../lvgl/ui.h"
#include "waveshare_rgb_lcd_port.h"
#include "fsaeEvents.h"
#include "rtc.h"
#include "sdcard.h"

#define INPUT1_GPIO      GPIO_NUM_15
#define INPUT2_GPIO      GPIO_NUM_16

#define RTC_SDA_GPIO     GPIO_NUM_8
#define RTC_SCL_GPIO     GPIO_NUM_9

#define SD_MOSI_GPIO     GPIO_NUM_11
#define SD_SCK_GPIO      GPIO_NUM_12
#define SD_MISO_GPIO     GPIO_NUM_13

#endif //INC_CENTRAL_H