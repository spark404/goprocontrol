//
// Created by Hugo Trippaers on 19/01/2021.
//

#include <esp_log.h>
#include <esp_err.h>

#include "mavlink_handlers.h"
#include "mavlink_camera.h"

static const char *TAG = "mavlink_camera";

static mavlink_camera_callbacks_t *_mavlink_camera_callbacks;

esp_err_t mavlink_camera_init(mavlink_camera_callbacks_t *mavlink_camera_callbacks)
{
    _mavlink_camera_callbacks = mavlink_camera_callbacks;

    return ESP_OK;
}

esp_err_t mavlink_camera_send_camera_information()
{
    if (_mavlink_camera_callbacks->camera_information_cb == NULL) {
        ESP_LOGD(TAG, "Handler for camera_information not defined");
        return ESP_FAIL;
    }

    mavlink_camera_information_t mavlink_camera_information;
    if (_mavlink_camera_callbacks->camera_information_cb(&mavlink_camera_information) != ESP_OK) {
        ESP_LOGD(TAG, "Handler for camera_information failed");
        return ESP_FAIL;
    }

    mavlink_msg_camera_information_send_struct(MAVLINK_COMM_0, &mavlink_camera_information);
    return ESP_OK;
}

esp_err_t mavlink_camera_send_camera_settings()
{
    if (_mavlink_camera_callbacks->camera_settings_cb == NULL) {
        return ESP_FAIL;
    }

    mavlink_camera_settings_t mavlink_camera_settings;
    if(_mavlink_camera_callbacks->camera_settings_cb(&mavlink_camera_settings) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_msg_camera_settings_send_struct(MAVLINK_COMM_0, &mavlink_camera_settings);
    return ESP_OK;
}

esp_err_t mavlink_camera_send_camera_capture_status()
{
    if (_mavlink_camera_callbacks->camera_capture_status_cb == NULL) {
        return ESP_FAIL;
    }

    mavlink_camera_capture_status_t mavlink_camera_capture_status;
    if(_mavlink_camera_callbacks->camera_capture_status_cb(&mavlink_camera_capture_status) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_msg_camera_capture_status_send_struct(MAVLINK_COMM_0, &mavlink_camera_capture_status);
    return ESP_OK;
}

esp_err_t mavlink_camera_send_storage_information(uint8_t storage_id)
{
    if (_mavlink_camera_callbacks->storage_information_cb == NULL) {
        return ESP_FAIL;
    }

    mavlink_storage_information_t mavlink_storage_information;
    if(_mavlink_camera_callbacks->storage_information_cb(&mavlink_storage_information) != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_msg_storage_information_send_struct(MAVLINK_COMM_0, &mavlink_storage_information);
    return ESP_OK;
}

esp_err_t mavlink_camera_handle_start_video_recording()
{
    if (_mavlink_camera_callbacks->command_start_recording_cb == NULL) {
        return ESP_FAIL;
    }

    return _mavlink_camera_callbacks->command_start_recording_cb();
}

esp_err_t mavlink_camera_handle_stop_video_recording()
{
    if (_mavlink_camera_callbacks->command_stop_recording_cb == NULL) {
        return ESP_FAIL;
    }

    return _mavlink_camera_callbacks->command_stop_recording_cb();
}

esp_err_t mavlink_camera_handle_start_capture()
{
    if (_mavlink_camera_callbacks->command_start_capture_cb == NULL) {
        return ESP_FAIL;
    }

    return _mavlink_camera_callbacks->command_start_capture_cb();
}

esp_err_t mavlink_camera_handle_stop_capture()
{
    if (_mavlink_camera_callbacks->command_stop_capture_cb == NULL) {
        return ESP_FAIL;
    }

    return _mavlink_camera_callbacks->command_stop_capture_cb();
}

esp_err_t mavlink_camera_handle_set_mode(uint8_t mode)
{
    if (_mavlink_camera_callbacks->command_set_mode_cb == NULL) {
        return ESP_FAIL;
    }

    return _mavlink_camera_callbacks->command_set_mode_cb(mode);
}

esp_err_t mavlink_camera_send_camera_image_captured(mavlink_camera_image_captured_t *camera_image_captured)
{
    mavlink_msg_camera_image_captured_send_struct(MAVLINK_COMM_0, camera_image_captured);
    return ESP_OK;
}

esp_err_t mavlink_camera_handle_digicam_control(float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    if (p5 >= 1) {
        mavlink_camera_settings_t mavlink_camera_settings;
        if(_mavlink_camera_callbacks->camera_settings_cb(&mavlink_camera_settings) != ESP_OK) {
            return ESP_FAIL;
        }

        if(mavlink_camera_settings.mode_id != CAMERA_MODE_IMAGE) {
            mavlink_camera_handle_set_mode(CAMERA_MODE_IMAGE);
        }

        ESP_LOGD(TAG, "digicam_control - request shot");
        return mavlink_camera_handle_start_capture();
    }

    return ESP_OK;
}

esp_err_t mavlink_camera_handle_digicam_configure(float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{

    return ESP_OK;
}