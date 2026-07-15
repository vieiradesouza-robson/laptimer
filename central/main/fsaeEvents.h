#ifndef INC_FSAEEVENTS_H
#define INC_FSAEEVENTS_H

#include "central.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "../../lvgl/screens/ui_skidScreen.h"
#include "../../lvgl/screens/ui_accelScreen.h"

void fsaeSkid_init(void);
void fsaeSkid_reset(void);

#endif //INC_CENTRAL_H