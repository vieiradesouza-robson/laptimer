#ifndef INC_FSAEEVENTS_H
#define INC_FSAEEVENTS_H

#include "central.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "../../lvgl/screens/ui_skidScreen.h"
#include "../../lvgl/screens/ui_accelScreen.h"

#define INPUT_TIME_MIN_INT_US 1500000 // 1500 ms debounce time for skidpad and acceleration inputs

void fsaeSkid_init(void);
void fsaeSkid_reset(void);
void fsaeSkid_deinit(void);

void fsaeAccel_init(void);
void fsaeAccel_reset(void);
void fsaeAccel_deinit(void);

#endif //INC_FSAEEVENTS_H