#include "photogate.h"

static const char *TAG = "DEVICE1";

// GPIO configuration
#define SWITCH_GPIO      GPIO_NUM_9
#define ESP_NOW_CHANNEL  1

// ESP-NOW peer MAC address (replace with Device 2's MAC)
static uint8_t peer_mac[] = {0x84, 0xF7, 0x03, 0x67, 0x61, 0x40};

// Message structure
typedef struct {
    uint32_t sequence;
    uint64_t timestamp;
    uint8_t command;  // 0 = toggle, 1 = set high, 2 = set low
} espnow_message_t;

static QueueHandle_t gpio_evt_queue = NULL;
static uint32_t sequence_number = 0;

// GPIO interrupt handler
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    gpio_isr_handler_remove(gpio_num);
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    portYIELD_FROM_ISR();
}

// GPIO task to handle events
static void gpio_task(void *arg) {
    uint32_t io_num;
    
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // Debounce - ignore multiple triggers within 50ms
            // vTaskDelay(pdMS_TO_TICKS(50));
            
            // Read GPIO state to confirm it's still low
            // if (gpio_get_level(io_num) == 0) {
                // ESP_LOGI(TAG, "Switch pressed, sending ESP-NOW command");
                
                // Prepare message
                espnow_message_t message;
                message.sequence = sequence_number++;
                message.timestamp = esp_timer_get_time();
                message.command = 0;  // Toggle command
                
                // Send via ESP-NOW
                esp_err_t result = esp_now_send(peer_mac, (uint8_t *)&message, sizeof(message));
                
                if (result == ESP_OK) {
                    ESP_LOGI(TAG, "ESP-NOW send success, sequence: %lu", message.sequence);
                } else {
                    ESP_LOGE(TAG, "ESP-NOW send failed: %d", result);
                }
            // }
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_isr_handler_add(SWITCH_GPIO, gpio_isr_handler, (void *)SWITCH_GPIO);
        }
    }
}

// ESP-NOW send callback
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send callback: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Initialize NVS
static void nvs_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// WiFi and ESP-NOW initialization
static void wifi_init(void) {
    // Initialize NVS first
    nvs_init();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void esp_now_initialize(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));
    
    // Add peer
    esp_now_peer_info_t peer_info = {
        .channel = ESP_NOW_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    };
    memcpy(peer_info.peer_addr, peer_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Device 1 (Switch Controller)");
    
    // Create queue for GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // GPIO configuration
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SWITCH_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Active low, so use pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE    // Negative edge (high to low)
    };
    gpio_config(&io_conf);
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SWITCH_GPIO, gpio_isr_handler, (void *)SWITCH_GPIO);
    
    // Initialize WiFi and ESP-NOW
    wifi_init();
    esp_now_initialize();
    
    // Create task to handle GPIO events
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "Device 1 ready. Switch connected to GPIO%d", SWITCH_GPIO);
    ESP_LOGI(TAG, "Will send toggle commands to: " MACSTR, MAC2STR(peer_mac));
}