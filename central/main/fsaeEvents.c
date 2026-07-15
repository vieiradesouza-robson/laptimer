#include "fsaeEvents.h"

static QueueHandle_t interrupt_queue = NULL;

uint64_t last_timestamp = 0;
uint64_t curr_timestamp = 0;
bool interrupt_enabled = false;

void enableInterrupt(){
    if(!interrupt_enabled){
        gpio_intr_enable(INPUT1_GPIO);
        interrupt_enabled = true;
    }
}

void disableInterrupt(){
    if(interrupt_enabled){
        gpio_intr_disable(INPUT1_GPIO);
        interrupt_enabled = false;
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {

    uint64_t timestamp = esp_timer_get_time();

    if (timestamp - last_timestamp < INPUT_TIME_MIN_INT_US) {
        return;
    }

    // Disable interrupt temporarily
    disableInterrupt();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(interrupt_queue, &timestamp, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void interrupt_task(void* arg) {

    BaseType_t newLap = pdFALSE;
    uint64_t diff_us = 0;
    float diff_s = 0.0f;

    while (1) {        
        newLap = xQueueReceive(interrupt_queue, &curr_timestamp, pdMS_TO_TICKS(50));

        if (!newLap) {
            curr_timestamp = esp_timer_get_time();
        }

        diff_us = curr_timestamp - last_timestamp;
        diff_s = diff_us / 1000000.0f;
        ui_skidNewTime(diff_s, (bool)(newLap != pdFALSE));

        last_timestamp = newLap ? curr_timestamp : last_timestamp;

        if (!interrupt_enabled && diff_us >= 1000000) {
            enableInterrupt();
        }
    }

    vTaskDelete(NULL);
}

void fsaeSkid_reset(void) {
    // Reset any internal state if necessary
    last_timestamp = 0;
    curr_timestamp = 0;
}

void fsaeSkid_init(void) {
    interrupt_queue = xQueueCreate(10, sizeof(uint64_t));
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << INPUT1_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT1_GPIO, gpio_isr_handler, NULL);

    xTaskCreate(interrupt_task, "interrupt_task", 2048, NULL, 10, NULL);
}
