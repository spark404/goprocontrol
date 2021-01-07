//
// Created by Hugo Trippaers on 04/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_CAMERA_CALLBACK_H
#define GOPROCONTROL_MAVLINK_CAMERA_CALLBACK_H

typedef enum {
    CMD_SET_MODE,
    CMD_GET_STATUS,
    CMD_GET_SETTINGS
} mavlink_camera_callback_type_t;

typedef enum {
    RECORDING_MODE_STILL,
    RECORDING_MODE_VIDEO
} recording_mode_t;

typedef struct {
    recording_mode_t mode;
} cmd_set_mode_t;

typedef struct {
    int ready;
} cmd_get_status_t;

typedef struct {
    recording_mode_t recording_mode;
    int processing;
    int recording_time_ms;
    float available_capacity;
    int image_count;
} cmd_get_settings_t;

typedef struct {
    mavlink_camera_callback_type_t type;
    union {
        cmd_set_mode_t cmd_set_mode;
    };
} mavlink_camera_callback_t;

typedef int32_t mavlink_camera_err_t;
#define CAMERA_OK 0
#define CAMERA_FAIL 1
#define CAMERA_INVALID_ARG 2

typedef mavlink_camera_err_t (*mavlink_camera_callback_handler_t)(const mavlink_camera_callback_t callback, void *pvParameters);

#endif //GOPROCONTROL_MAVLINK_CAMERA_CALLBACK_H
