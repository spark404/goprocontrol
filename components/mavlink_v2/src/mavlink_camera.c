//
// Created by Hugo Trippaers on 19/01/2021.
//

#include <esp_log.h>
#include <esp_err.h>

#include "mavlink_handlers.h"
#include "mavlink_camera.h"

static const char *TAG = "mavlink_camera";

esp_err_t mavlink_camera_send_camera_information(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.camera_information_cb == NULL) {
        ESP_LOGD(TAG, "Handler for camera_information not defined");
        return ESP_FAIL;
    }

    mavlink_camera_information_t mavlink_camera_information;
    if (mavlink_camera_callbacks.camera_information_cb(&mavlink_camera_information) != ESP_OK) {
        ESP_LOGD(TAG, "Handler for camera_information failed");
        return ESP_FAIL;
    }

    mavlink_msg_camera_information_send_struct(MAVLINK_COMM_0, &mavlink_camera_information);
    return ESP_OK;
}

esp_err_t mavlink_camera_send_camera_settings(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.camera_settings_cb == NULL) {
        return ESP_FAIL;
    }

    mavlink_camera_settings_t mavlink_camera_settings;
    if(mavlink_camera_callbacks.camera_settings_cb(&mavlink_camera_settings) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_msg_camera_settings_send_struct(MAVLINK_COMM_0, &mavlink_camera_settings);
    return ESP_OK;
}

esp_err_t mavlink_camera_send_camera_capture_status(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.camera_capture_status_cb == NULL) {
        return ESP_FAIL;
    }

    mavlink_camera_capture_status_t mavlink_camera_capture_status;
    if(mavlink_camera_callbacks.camera_capture_status_cb(&mavlink_camera_capture_status) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_msg_camera_capture_status_send_struct(MAVLINK_COMM_0, &mavlink_camera_capture_status);
    return ESP_OK;
}

esp_err_t mavlink_camera_send_storage_information(mavlink_camera_callbacks_t mavlink_camera_callbacks, uint8_t storage_id)
{
    if (mavlink_camera_callbacks.storage_information_cb == NULL) {
        return ESP_FAIL;
    }

    mavlink_storage_information_t mavlink_storage_information;
    if(mavlink_camera_callbacks.storage_information_cb(&mavlink_storage_information) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_msg_storage_information_send_struct(MAVLINK_COMM_0, &mavlink_storage_information);
    return ESP_OK;
}

esp_err_t mavlink_camera_handle_start_video_recording(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.command_start_recording_cb == NULL) {
        return ESP_FAIL;
    }

    return mavlink_camera_callbacks.command_start_recording_cb();
}

esp_err_t mavlink_camera_handle_stop_video_recording(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.command_stop_recording_cb == NULL) {
        return ESP_FAIL;
    }

    return mavlink_camera_callbacks.command_stop_recording_cb();
}

esp_err_t mavlink_camera_handle_start_capture(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.command_start_capture_cb == NULL) {
        return ESP_FAIL;
    }

    return mavlink_camera_callbacks.command_start_capture_cb();
}

esp_err_t mavlink_camera_handle_stop_capture(mavlink_camera_callbacks_t mavlink_camera_callbacks)
{
    if (mavlink_camera_callbacks.command_stop_capture_cb == NULL) {
        return ESP_FAIL;
    }

    return mavlink_camera_callbacks.command_stop_capture_cb();
}

esp_err_t mavlink_camera_handle_set_mode(mavlink_camera_callbacks_t mavlink_camera_callbacks, uint8_t mode)
{
    if (mavlink_camera_callbacks.command_set_mode_cb == NULL) {
        return ESP_FAIL;
    }

    return mavlink_camera_callbacks.command_set_mode_cb(mode);
}

esp_err_t mavlink_camera_send_camera_image_captured(mavlink_camera_image_captured_t *camera_image_captured)
{
    mavlink_msg_camera_image_captured_send_struct(MAVLINK_COMM_0, camera_image_captured);
    return ESP_OK;
}