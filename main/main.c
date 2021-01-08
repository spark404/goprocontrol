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
#include "mavlink_v2.h"
#include "gopro.h"
#include "mavlink_camera_callback.h"

#define TAG "app_main"

static gopro_connection_t connection;

mavlink_camera_err_t camera_callback(mavlink_camera_callback_t callback, void *pvParameters) {
    mavlink_camera_err_t result = CAMERA_FAIL;
    gopro_status_t status;

    switch (callback.type) {
        case CMD_SET_MODE:
            switch (callback.cmd_set_mode.mode) {
                case RECORDING_MODE_STILL:
                    if (gopro_setmode(&connection, GOPRO_STILL) == ESP_OK) {
                        result = CAMERA_OK;
                    }
                    break;
                case RECORDING_MODE_VIDEO:
                    if (gopro_setmode(&connection, GOPRO_VIDEO) == ESP_OK) {
                        result = CAMERA_OK;
                    }
                    break;
                default:
                    ESP_LOGW(TAG, "No gopro mode for mode type %d", callback.cmd_set_mode.mode);
                    result = CAMERA_INVALID_ARG;
                    break;
            }
            break;
        case CMD_GET_STATUS:
            if (pvParameters == NULL) {
                result = CAMERA_INVALID_ARG;
                break;
            }

            if (gopro_get_status(&connection, &status) != ESP_OK) {
                result = CAMERA_FAIL;
                break;
            }

            cmd_get_status_t *cmd_get_status = pvParameters;
            cmd_get_status->ready = status.ready;
            result = CAMERA_OK;
            break;
        case CMD_GET_SETTINGS:
            if (pvParameters == NULL) {
                result = CAMERA_INVALID_ARG;
                break;
            }

            gopro_settings_t gopro_settings;
            if (gopro_get_settings(&connection, &gopro_settings) != ESP_OK) {
                result = CAMERA_FAIL;
                break;
            }

            cmd_get_settings_t *cmd_get_settings = pvParameters;
            if (gopro_settings.mode == GOPRO_STILL) {
                cmd_get_settings->recording_mode = RECORDING_MODE_STILL;
            } else if (gopro_settings.mode == GOPRO_VIDEO) {
                cmd_get_settings->recording_mode = RECORDING_MODE_VIDEO;
            } else {
                ESP_LOGW(TAG, "GoPro is in a mode that has no equivalent in MavLink");
                cmd_get_settings->recording_mode = RECORDING_MODE_STILL; //FIXME
            }

            cmd_get_settings->image_count = gopro_settings.num_photos_taken;
            cmd_get_settings->available_capacity = gopro_settings.remaining_capacity_bytes;
            cmd_get_settings->recording_time_ms = gopro_settings.recording_duration * 1000;
            cmd_get_settings->processing = gopro_settings.processing;

            result = CAMERA_OK;
            break;
        case CMD_START_VIDEO_RECORDING:
            result = gopro_start_recording(&connection) == ESP_OK ? CAMERA_OK : CAMERA_FAIL;
            break;
        case CMD_STOP_VIDEO_RECORDING:
            result = gopro_stop_recording(&connection) == ESP_OK ? CAMERA_OK : CAMERA_FAIL;
            break;
        case CMD_START_STILL_RECORDING:
            result = gopro_start_recording(&connection) == ESP_OK ? CAMERA_OK : CAMERA_FAIL;
            break;
        default:
            ESP_LOGW(TAG, "No defined action for callback %d", callback.type);
    }

    ESP_LOGD(TAG, "Camera callback handler complete for cmd %d (result: %d)", callback.type, result);
    return result;
}

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
    esp_log_level_set("HTTP_CLIENT", ESP_LOG_INFO);
    esp_log_level_set("gopro_http", ESP_LOG_DEBUG);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Event Loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xTaskHandle xBlinkTaskHandle;

    // Create the blink task
    if (xTaskCreate(&blink_task, (const char *) "BlinkTask", 2048, NULL, tskIDLE_PRIORITY + 10, &xBlinkTaskHandle) !=
            pdPASS) {
        ESP_LOGE(TAG, "Blink Task create failed");
    }
    ESP_LOGI(TAG, "[APP] Blink Running..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());

    ESP_LOGI(TAG, "[APP] Starting GOPRO on WiFI");
    ESP_ERROR_CHECK(gopro_init(&connection));

    ESP_LOGI(TAG, "[APP] Starting MAVLINK on UART");
    ESP_ERROR_CHECK(mavlink_v2_init(camera_callback));
    ESP_ERROR_CHECK(enable_mavlink_on_uart());

    // No need to start the scheduler, that is taken care of by the ESP SDK
    ESP_LOGI(TAG, "Completed app_main, keep on going on");

    for(;;) {
        vTaskDelay((portTickType) portMAX_DELAY);
    }
}

