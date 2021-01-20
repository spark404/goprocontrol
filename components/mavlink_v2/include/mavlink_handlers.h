//
// Created by Hugo Trippaers on 19/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_HANDLERS_H
#define GOPROCONTROL_MAVLINK_HANDLERS_H

#include <freertos/FreeRTOS.h>
#include "freertos/queue.h"

#include "mavlink_bridge.h"

typedef enum {
    MAVLINK_COMPONENT_STATUS_NOT_READY,
    MAVLINK_COMPONENT_STATUS_READY,
    MAVLINK_COMPONENT_STATUS_UNKNOWN,
} mavlink_component_status_t;

typedef mavlink_component_status_t (*mavlink_status_callback_t)();

typedef esp_err_t (*mavlink_camera_information_cb_t)(mavlink_camera_information_t *mavlink_camera_information);
typedef esp_err_t (*mavlink_camera_settings_cb_t)(mavlink_camera_settings_t *mavlink_camera_settings);
typedef esp_err_t (*mavlink_camera_capture_status_cb_t)(mavlink_camera_capture_status_t *mavlink_camera_capture_status);
typedef esp_err_t (*mavlink_storage_information_cb_t)(mavlink_storage_information_t *mavlink_storage_information);

typedef esp_err_t (*mavlink_camera_command_set_mode_t)(int mode);
typedef esp_err_t (*mavlink_camera_command_start_recording_t)();
typedef esp_err_t (*mavlink_camera_command_stop_recording_t)();
typedef esp_err_t (*mavlink_camera_command_start_capture_t)();
typedef esp_err_t (*mavlink_camera_command_stop_capture_t)();

typedef struct {
    mavlink_camera_information_cb_t camera_information_cb;
    mavlink_camera_settings_cb_t camera_settings_cb;
    mavlink_camera_capture_status_cb_t camera_capture_status_cb;
    mavlink_storage_information_cb_t storage_information_cb;
    mavlink_camera_command_set_mode_t command_set_mode_cb;
    mavlink_camera_command_start_recording_t command_start_recording_cb;
    mavlink_camera_command_stop_recording_t command_stop_recording_cb;
    mavlink_camera_command_start_capture_t command_start_capture_cb;
    mavlink_camera_command_stop_capture_t command_stop_capture_cb;
} mavlink_camera_callbacks_t;

typedef struct mavlink_config {
    QueueHandle_t incoming_queue;
    mavlink_status_callback_t status_callback;
    mavlink_camera_callbacks_t camera_callbacks;
} *mavlink_config_t;

uint32_t get_time_since_boot_in_ms();
esp_err_t mavlink_handler_configure(mavlink_config_t mavlink_config);

esp_err_t mavlink_camera_send_camera_image_captured(mavlink_camera_image_captured_t *camera_image_captured);

#endif //GOPROCONTROL_MAVLINK_HANDLERS_H
