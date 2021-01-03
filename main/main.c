#include <stdint.h>
//
// Created by Hugo Trippaers on 12/09/2020.
//
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <nvs_flash.h>
#include <esp_event.h>

#include "sdkconfig.h"

#include "blink.h"
#include "wifi.h"
#include "mavlink_v2.h"

#define TAG "app_main"

_Noreturn void app_main(void) {
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "[APP] Running from partition: %s", running_partition->label);

    const esp_app_desc_t *appDescription = esp_ota_get_app_description();
    ESP_LOGI(TAG, "[APP] Running: %s (version: %s)", appDescription->project_name, appDescription->version);

    esp_log_level_set("blink_task", ESP_LOG_WARN);
    esp_log_level_set("mavlink_uart", ESP_LOG_DEBUG);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Event Loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Connect to WiFi (station mode)
    wifi_connect();
    ESP_LOGI(TAG, "[APP] Wifi Connected..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());

    xTaskHandle xBlinkTaskHandle;

    // Create the blink task
    if (xTaskCreate(&blink_task, (const char *) "BlinkTask", 2048, NULL, tskIDLE_PRIORITY + 10, &xBlinkTaskHandle) !=
            pdPASS) {
        ESP_LOGE(TAG, "Blink Task create failed");
    }
    ESP_LOGI(TAG, "[APP] Blink Running..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());

    ESP_LOGI(TAG, "[APP] Starting MAVLINK on UART");
    ESP_ERROR_CHECK(enable_mavlink_on_uart());

    // No need to start the scheduler, that is taken care of by the ESP SDK
    ESP_LOGI(TAG, "Completed app_main, keep on going on");

    for(;;) {
        vTaskDelay((portTickType) portMAX_DELAY);
    }
}