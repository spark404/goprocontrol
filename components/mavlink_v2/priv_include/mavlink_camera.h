//
// Created by Hugo Trippaers on 19/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_CAMERA_H
#define GOPROCONTROL_MAVLINK_CAMERA_H

#include "mavlink_bridge.h"

esp_err_t mavlink_camera_init(mavlink_camera_callbacks_t *mavlink_camera_callbacks);

esp_err_t mavlink_camera_send_camera_information();
esp_err_t mavlink_camera_send_camera_settings();
esp_err_t mavlink_camera_send_camera_capture_status();
esp_err_t mavlink_camera_send_storage_information(uint8_t storage_id);

esp_err_t mavlink_camera_handle_set_mode(uint8_t mode);
esp_err_t mavlink_camera_handle_start_video_recording();
esp_err_t mavlink_camera_handle_stop_video_recording();
esp_err_t mavlink_camera_handle_start_capture();
esp_err_t mavlink_camera_handle_stop_capture();

esp_err_t mavlink_camera_handle_digicam_control(float p1, float p2, float p3, float p4, float p5, float p6, float p7);
esp_err_t mavlink_camera_handle_digicam_configure(float p1, float p2, float p3, float p4, float p5, float p6, float p7);

#endif //GOPROCONTROL_MAVLINK_CAMERA_H
