//
// Created by Hugo Trippaers on 12/09/2020.
//
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <nvs_flash.h>
#include <esp_event.h>

#include "sdkconfig.h"

#include "blink.h"
#include "gopro.h"

#include "mavlink_handlers.h"
#include "mavlink_uart.h"

#define TAG "app_main"

static gopro_connection_t connection;

// callbacks
mavlink_component_status_t camera_status();
esp_err_t camera_information(mavlink_camera_information_t *mavlink_camera_information);
esp_err_t camera_settings(mavlink_camera_settings_t *mavlink_camera_settings);
esp_err_t storage_information(mavlink_storage_information_t *storage_information);
esp_err_t camera_capture_status(mavlink_camera_capture_status_t *camera_capture_status);
esp_err_t command_set_mode(int mode);
esp_err_t command_start_recording();
esp_err_t command_stop_recording();
esp_err_t command_start_capture();
esp_err_t command_stop_capture();

blink_status_t blink_status();

static xTimerHandle captureTimer;
void capture_timer_cb(TimerHandle_t xTimer);

_Noreturn void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "[APP] Running from partition: %s", running_partition->label);

    const esp_app_desc_t *appDescription = esp_ota_get_app_description();
    ESP_LOGI(TAG, "[APP] Running: %s (version: %s)", appDescription->project_name, appDescription->version);

#if CONFIG_LOG_DEFAULT_LEVEL >= 4 // DEBUG
    esp_log_level_set("*", ESP_LOG_WARN);

    esp_log_level_set("app_main", ESP_LOG_DEBUG);

    esp_log_level_set("gopro", ESP_LOG_DEBUG);
    esp_log_level_set("gopro_http", ESP_LOG_DEBUG);
    esp_log_level_set("gopro_wifi", ESP_LOG_DEBUG);
    esp_log_level_set("gopro_wol", ESP_LOG_DEBUG);

    esp_log_level_set("mavlink_uart", ESP_LOG_DEBUG);
    esp_log_level_set("mavlink_v2", ESP_LOG_DEBUG);
    esp_log_level_set("mavlink_camera", ESP_LOG_DEBUG);
#endif

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
    blink_config_t blink_config = {
            .default_delay_in_ms = 500,
            .status_cb = blink_status
    };
    if (xTaskCreate(&blink_task, (const char *) "BlinkTask", 2048, &blink_config, tskIDLE_PRIORITY + 10, &xBlinkTaskHandle) !=
            pdPASS) {
        ESP_LOGE(TAG, "Blink Task create failed");
    }
    ESP_LOGI(TAG, "[APP] Blink Running..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());

    ESP_LOGI(TAG, "[APP] Starting GOPRO on WiFI");
    ESP_ERROR_CHECK(gopro_init(&connection));

    ESP_LOGI(TAG, "[APP] Starting MAVLINK on UART");
    mavlink_uart_handle_t mavlink_uart_handle = mavlink_uart_configure();
    assert(mavlink_uart_handle != NULL);
    ESP_ERROR_CHECK(mavlink_uart_connect(mavlink_uart_handle));

    ESP_LOGI(TAG, "[APP] Starting MAVLINK handlers");
    struct mavlink_config config = {
            .incoming_queue = mavlink_uart_handle->incoming_message_queue,
            .status_callback = camera_status,
            .camera_callbacks = {
                    .camera_information_cb = camera_information,
                    .camera_settings_cb = camera_settings,
                    .storage_information_cb = storage_information,
                    .camera_capture_status_cb = camera_capture_status,
                    .command_set_mode_cb = command_set_mode,
                    .command_start_capture_cb = command_start_capture,
                    .command_stop_capture_cb = command_stop_capture,
                    .command_start_recording_cb = command_start_recording,
                    .command_stop_recording_cb = command_stop_recording,
            }
    };
    ESP_ERROR_CHECK(mavlink_handler_configure(&config));

    // No need to start the scheduler, that is taken care of by the ESP SDK
    ESP_LOGI(TAG, "Completed app_main, keep on going on");

    for (;;) {
        vTaskDelay((portTickType) portMAX_DELAY);
    }
}

// Blink callback
blink_status_t blink_status()
{
    if (connection.connection_state == -1) {
        return BLINK_STATUS_INITIALIZING;
    }

    gopro_status_t status;
    if (gopro_get_status(&connection, &status) != ESP_OK) {
        return BLINK_STATUS_ERROR;
    };

    return status.ready ? BLINK_STATUS_READY : BLINK_STATUS_INITIALIZING;
}

// Callbacks
mavlink_component_status_t camera_status()
{
    gopro_status_t status;
    if (gopro_get_status(&connection, &status) != ESP_OK) {
        return MAVLINK_COMPONENT_STATUS_NOT_READY;
    };

    return status.ready ? MAVLINK_COMPONENT_STATUS_READY : MAVLINK_COMPONENT_STATUS_NOT_READY;
}

esp_err_t camera_information(mavlink_camera_information_t *mavlink_camera_information)
{
    if (mavlink_camera_information == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gopro_status_t status;
    if (gopro_get_status(&connection, &status) != ESP_OK) {
        return ESP_FAIL;
    };

    mavlink_camera_information->time_boot_ms = get_time_since_boot_in_ms();

    uint32_t flags =
        CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
        CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
        CAMERA_CAP_FLAGS_HAS_MODES |
        CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM;

    strncpy((char *) mavlink_camera_information->vendor_name, "GoPro", 32);
    strncpy((char *) mavlink_camera_information->model_name, status.camera_name, 32);

    mavlink_camera_information->firmware_version = 1;
    mavlink_camera_information->focal_length = 16.4f;
    mavlink_camera_information->sensor_size_h = 1.1f;
    mavlink_camera_information->sensor_size_v = 1.1f;
    mavlink_camera_information->resolution_h = 1920;
    mavlink_camera_information->resolution_v = 1080;
    mavlink_camera_information->lens_id = 0;
    mavlink_camera_information->flags = flags;
    mavlink_camera_information->cam_definition_version = 0;
    mavlink_camera_information->cam_definition_uri[0] = 0x0;

    return ESP_OK;
}

esp_err_t camera_settings(mavlink_camera_settings_t *mavlink_camera_settings)
{
    if (mavlink_camera_settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_camera_settings->time_boot_ms = get_time_since_boot_in_ms();
    switch (settings.mode) {
        case GOPRO_VIDEO:
            mavlink_camera_settings->mode_id = CAMERA_MODE_VIDEO;
            break;
        case GOPRO_STILL:
            mavlink_camera_settings->mode_id = CAMERA_MODE_IMAGE;
            break;
        case GOPRO_MULTISHOT:
            // We should switch to a supported mode here
            mavlink_camera_settings->mode_id = CAMERA_MODE_IMAGE;
            break;
    }

    mavlink_camera_settings->focusLevel = 0;
    mavlink_camera_settings->zoomLevel = 0;

    return ESP_OK;
}

esp_err_t storage_information(mavlink_storage_information_t *storage_information)
{
    if (storage_information == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    storage_information->storage_id = 1;
    storage_information->storage_count = 1;
    storage_information->time_boot_ms = get_time_since_boot_in_ms();
    storage_information->status = STORAGE_STATUS_READY;
    storage_information->type = STORAGE_TYPE_MICROSD;
    storage_information->available_capacity = settings.remaining_capacity_bytes / (1024 * 1024);

    // We don't know, set to zero and see what happens
    storage_information->total_capacity = 0.0f;
    storage_information->used_capacity = 0.0f;
    storage_information->read_speed = 0.0f;
    storage_information->write_speed = 0.0f;

    return ESP_OK;
}

esp_err_t camera_capture_status(mavlink_camera_capture_status_t *camera_capture_status)
{
    if (camera_capture_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    camera_capture_status->time_boot_ms = get_time_since_boot_in_ms();
    camera_capture_status->recording_time_ms = settings.recording_duration * 1000;
    camera_capture_status->available_capacity = settings.remaining_capacity_bytes / (1024 * 1024);
    camera_capture_status->image_count = settings.num_photos_taken;
    camera_capture_status->image_interval = 0.0f;
    camera_capture_status->video_status = settings.mode == 0 && settings.processing;
    camera_capture_status->image_status = settings.mode == 1 && settings.processing;

    return ESP_OK;
}

esp_err_t command_set_mode(int mode)
{
    gopro_mode_t gopro_mode;

    switch (mode) {
        case CAMERA_MODE_IMAGE:
            gopro_mode = GOPRO_STILL;
            break;
        case CAMERA_MODE_VIDEO:
            gopro_mode = GOPRO_VIDEO;
            break;
        default:
            gopro_mode = GOPRO_STILL;
            break;
    }

    return gopro_setmode(&connection, gopro_mode);
}

esp_err_t command_start_recording()
{
    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    if (settings.mode != GOPRO_VIDEO) {
        return ESP_FAIL;
    }

    return gopro_start_recording(&connection);
}

esp_err_t command_stop_recording()
{
    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    if (settings.mode != GOPRO_VIDEO) {
        return ESP_FAIL;
    }

    return gopro_stop_recording(&connection);
}

esp_err_t command_start_capture()
{
    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    if (settings.mode != GOPRO_STILL) {
        return ESP_FAIL;
    }

    esp_err_t result = gopro_start_recording(&connection);

    if (result == ESP_OK) {
        captureTimer = xTimerCreate("captureTimer", 250 / portTICK_PERIOD_MS, pdFALSE, NULL, capture_timer_cb);
        xTimerStart(captureTimer, 0);
    }

    return result;
}

esp_err_t command_stop_capture()
{
    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        return ESP_FAIL;
    }

    if (settings.mode != GOPRO_VIDEO) {
        return ESP_FAIL;
    }

    return ESP_OK; // Nothing to do yet
}

void capture_timer_cb(TimerHandle_t xTimer) {
    gopro_settings_t settings;
    if (gopro_get_settings(&connection, &settings) != ESP_OK) {
        ESP_LOGD(TAG, "[Capture Timer] retrieval of settings failed");
        xTimerDelete(xTimer, 0);
        return;
    }

    if (settings.processing == 1) {
        ESP_LOGD(TAG, "[Capture Timer] camera still processing");
        xTimerReset(xTimer, 0);
        return;
    }

    ESP_LOGD(TAG, "[Capture Timer] sending CAMERA_IMAGE_CAPTURED");
    mavlink_camera_image_captured_t camera_image_captured = {
            .time_boot_ms = get_time_since_boot_in_ms(),
            .image_index = -1, // New image
            .capture_result = 1, // 1 = OK
            .lat = 0,
            .lon = 0,
            .alt = 0,
            .relative_alt = 0,
            .q = {0, 0, 0, 0},
            .file_url = { 0x0 }
    };

    mavlink_camera_send_camera_image_captured(&camera_image_captured);
    xTimerDelete(xTimer, 0);
}