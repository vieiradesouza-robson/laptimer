#include "fsaeEvents.h"

static QueueHandle_t interrupt_queue = NULL;
static TaskHandle_t skid_task_handle = NULL;
static TaskHandle_t status_checkbox_task_handle = NULL;

float skidTimes[4] = {0.0, 0.0, 0.0, 0.0};
uint16_t skidCurrentIndex = 0;

uint64_t last_timestamp = 0;
uint64_t curr_timestamp = 0;
bool interrupt_enabled = false;

skid_button_status_t skid_button_status = BUTTON_START;

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

static void skid_interrupt_task(void* arg) {

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
        skidTimes[skidCurrentIndex] = diff_s;
        ui_skidNewTime(diff_s, skidCurrentIndex);

        last_timestamp = newLap ? curr_timestamp : last_timestamp;
        skidCurrentIndex = newLap ? (skidCurrentIndex + 1) % 4 : skidCurrentIndex;

        if (skidCurrentIndex == 0 && newLap) {
            // Calculate average of 2nd and 4th laps
            float avg = (skidTimes[1] + skidTimes[3]) / 2.0f;
            ui_skidNewTime(avg, 4);
        }

        if (!interrupt_enabled && diff_us >= INPUT_TIME_MIN_INT_US) {
            enableInterrupt();
        }
    }

    vTaskDelete(NULL);
}

static void update_photogate_status_skid(void* arg){
    while (1) {
        // Update the photogate status checkbox based on the current photogate pin state
        int photogate_state = gpio_get_level(INPUT1_GPIO);
        ui_skidPhotogateStatus(photogate_state);
        vTaskDelay(pdMS_TO_TICKS(200)); // Add a small delay to avoid busy waiting
    }
}

void fsaeSkid_reset(void) {
    skidCurrentIndex = 0;
    // Reset any internal state if necessary
    last_timestamp = 0;
    curr_timestamp = 0;
}

void fsaeSkid_button_pressed(void) {
    switch (skid_button_status) {
        case BUTTON_START:
            xTaskCreate(skid_interrupt_task, "skid_interrupt_task", 2048, NULL, 10, &skid_task_handle);
            skid_button_status = BUTTON_STOP;
            break;
        case BUTTON_STOP:
            vTaskDelete(skid_task_handle);
            skid_button_status = BUTTON_RESET;
            break;
        case BUTTON_RESET:
            fsaeSkid_reset();
            skid_button_status = BUTTON_START;
            break;
        default:
            break;
    }
    set_button_text(skid_button_status);
}

void fsaeSkid_init(void) {
    // Only one event can own the photogate inputs at a time
    fsaeAccel_deinit();
    // fsaeSkid_deinit();

    interrupt_queue = xQueueCreate(10, sizeof(uint64_t));
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << INPUT1_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // gpio_install_isr_service is idempotent-safe to call again; already-installed is ignored
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT1_GPIO, gpio_isr_handler, NULL);

    xTaskCreate(update_photogate_status_skid, "update_photogate_status_skid", 2048, NULL, 10, &status_checkbox_task_handle);
    fsaeSkid_reset();
}

void fsaeSkid_deinit(void) {
    disableInterrupt();
    gpio_isr_handler_remove(INPUT1_GPIO);

    if (skid_task_handle != NULL) {
        vTaskDelete(skid_task_handle);
        skid_task_handle = NULL;
    }

    if (status_checkbox_task_handle != NULL) {
        vTaskDelete(status_checkbox_task_handle);
        status_checkbox_task_handle = NULL;
    }

    if (interrupt_queue != NULL) {
        vQueueDelete(interrupt_queue);
        interrupt_queue = NULL;
    }
}

// ==================== Acceleration event ====================

typedef struct {
    gpio_num_t gpio;
    uint64_t timestamp;
} accel_event_t;

static QueueHandle_t accel_queue = NULL;
static TaskHandle_t accel_task_handle = NULL;
static TaskHandle_t accel_status_checkbox_task_handle = NULL;

static uint64_t accel_last_timestamp = 0;
static uint64_t accel_input1_timestamp = 0;
static bool accel_input1_intr_enabled = false;
static bool accel_input2_intr_enabled = false;
bool runOpen = false;

static void enableAccelInput1Interrupt(void) {
    if (!accel_input1_intr_enabled) {
        gpio_intr_enable(INPUT1_GPIO);
        accel_input1_intr_enabled = true;
    }
}

static void disableAccelInput1Interrupt(void) {
    if (accel_input1_intr_enabled) {
        gpio_intr_disable(INPUT1_GPIO);
        accel_input1_intr_enabled = false;
    }
}

static void enableAccelInput2Interrupt(void) {
    if (!accel_input2_intr_enabled) {
        gpio_intr_enable(INPUT2_GPIO);
        accel_input2_intr_enabled = true;
    }
}

static void disableAccelInput2Interrupt(void) {
    if (accel_input2_intr_enabled) {
        gpio_intr_disable(INPUT2_GPIO);
        accel_input2_intr_enabled = false;
    }
}

static void IRAM_ATTR accel_gpio_isr_handler(void* arg) {

    gpio_num_t gpio = (gpio_num_t)(intptr_t)arg;
    uint64_t timestamp = esp_timer_get_time();

    if (timestamp - accel_last_timestamp < INPUT_TIME_MIN_INT_US) {
        return;
    }

    if (gpio == INPUT1_GPIO) {
        gpio_intr_disable(INPUT1_GPIO);
        accel_input1_intr_enabled = false;
    } else {
        gpio_intr_disable(INPUT2_GPIO);
        accel_input2_intr_enabled = false;
    }

    accel_event_t evt = { .gpio = gpio, .timestamp = timestamp };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(accel_queue, &evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void accel_task(void* arg) {

    accel_event_t evt;
    uint64_t curr_timestamp = 0;
    BaseType_t runDone = pdFALSE;
    uint64_t diff_us = 0;
    float diff_s = 0.0f;

    while (1) {

        runDone = xQueueReceive(accel_queue, &evt, pdMS_TO_TICKS(50));

        if (!runDone) {
            if (runOpen){
                curr_timestamp = esp_timer_get_time();
                diff_us = curr_timestamp - accel_input1_timestamp;
                diff_s = diff_us / 1000000.0f;
                ui_accelNewTime(diff_s);
            }
            continue;
        }

        accel_last_timestamp = evt.timestamp;

        if (evt.gpio == INPUT1_GPIO) {
            accel_input1_timestamp = evt.timestamp;
            runOpen = true;
            enableAccelInput2Interrupt();
        } else {
            diff_us = evt.timestamp - accel_input1_timestamp;
            diff_s = diff_us / 1000000.0f;
            runOpen = false;

            ui_accelNewTime(diff_s);
            // enableAccelInput1Interrupt();
        }
    }

    vTaskDelete(NULL);
}

static void update_photogate_status_accel(void* arg){
    int photogate_state, photogate_state2;
    while (1) {
        // Update the photogate status checkbox based on the current photogate pin state
        photogate_state = gpio_get_level(INPUT1_GPIO);
        photogate_state2 = gpio_get_level(INPUT2_GPIO);
        ui_accelPhotogateStatus(photogate_state, photogate_state2);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void fsaeAccel_reset(void) {
    accel_last_timestamp = 0;
    accel_input1_timestamp = 0;

    if (accel_queue != NULL) {
        xQueueReset(accel_queue);
    }

    runOpen = false;
    ui_accelNewTime(0.0f);

    disableAccelInput2Interrupt();
    enableAccelInput1Interrupt();
}

void fsaeAccel_deinit(void) {
    disableAccelInput1Interrupt();
    disableAccelInput2Interrupt();
    gpio_isr_handler_remove(INPUT1_GPIO);
    gpio_isr_handler_remove(INPUT2_GPIO);

    if (accel_task_handle != NULL) {
        vTaskDelete(accel_task_handle);
        accel_task_handle = NULL;
    }

    if(accel_status_checkbox_task_handle != NULL) {
        vTaskDelete(accel_status_checkbox_task_handle);
        accel_status_checkbox_task_handle = NULL;
    }

    if (accel_queue != NULL) {
        vQueueDelete(accel_queue);
        accel_queue = NULL;
    }
}

void fsaeAccel_init(void) {
    // Only one event can own the photogate inputs at a time
    fsaeSkid_deinit();
    // fsaeAccel_deinit();

    accel_queue = xQueueCreate(10, sizeof(accel_event_t));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << INPUT1_GPIO) | (1ULL << INPUT2_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // gpio_install_isr_service is already called once by fsaeSkid_init() at boot
    gpio_isr_handler_add(INPUT1_GPIO, accel_gpio_isr_handler, (void*)(intptr_t)INPUT1_GPIO);
    gpio_isr_handler_add(INPUT2_GPIO, accel_gpio_isr_handler, (void*)(intptr_t)INPUT2_GPIO);

    xTaskCreate(accel_task, "accel_task", 2048, NULL, 10, &accel_task_handle);
    xTaskCreate(update_photogate_status_accel, "update_photogate_status_accel", 2048, NULL, 10, &accel_status_checkbox_task_handle);

    fsaeAccel_reset();
}
