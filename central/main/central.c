#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"

static const char *TAG = "DEVICE2";

// GPIO configuration
#define OUTPUT_GPIO      GPIO_NUM_3
#define ESP_NOW_CHANNEL  1

// Message structure (must match sender)
typedef struct {
    uint32_t sequence;
    uint64_t timestamp;
    uint8_t command;  // 0 = toggle, 1 = set high, 2 = set low
} espnow_message_t;

typedef struct {
    uint8_t mac_addr[6];
    espnow_message_t message;
} espnow_event_t;

static bool output_state = false;
static QueueHandle_t espnow_evt_queue = NULL;

// ESP-NOW receive callback
static void esp_now_recv_cb(const esp_now_recv_info_t *mac_addr, const uint8_t *data, int len) {
    if (len == sizeof(espnow_message_t)) {
        espnow_message_t *message = (espnow_message_t *)data;
        espnow_event_t event;
        memcpy(event.mac_addr, mac_addr->src_addr, 6);
        memcpy(&event.message, message, sizeof(espnow_message_t));

        xQueueSend(espnow_evt_queue, &event, portMAX_DELAY);
    } else {
        ESP_LOGE(TAG, "Invalid message length: %d", len);
    }
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
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
}

static void event_processing_task(void *pvParameter) {
    espnow_event_t last_event;
    espnow_event_t current_event;
    bool has_last = false;

    while (1) {
        if (xQueueReceive(espnow_evt_queue, &current_event, portMAX_DELAY)) {
            if (has_last) {
                int64_t diff = (current_event.message.timestamp - last_event.message.timestamp)/1000;
                ESP_LOGI(TAG, "delta t: %lld ms", diff);
                has_last = false;
            } else {
                has_last = true;
            }
            last_event = current_event;
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Device 2 (Output Controller)");

    // Create queue for GPIO events
    espnow_evt_queue = xQueueCreate(10, sizeof(espnow_event_t));
    
    // GPIO configuration for output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OUTPUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Initialize output to LOW
    output_state = false;
    gpio_set_level(OUTPUT_GPIO, output_state);
    
    // Initialize WiFi and ESP-NOW
    wifi_init();
    esp_now_initialize();

    xTaskCreate(event_processing_task, "event_processing_task", 2048, NULL, 5, NULL);
    
    // Print MAC address so Device 1 can be configured
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "Device 2 MAC: " MACSTR, MAC2STR(mac));
    ESP_LOGI(TAG, "Output on GPIO%d, initial state: LOW", OUTPUT_GPIO);
    ESP_LOGI(TAG, "Waiting for ESP-NOW commands...");
}