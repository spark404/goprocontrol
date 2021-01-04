//
// Created by Hugo Trippaers on 04/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_CAMERA_CALLBACK_H
#define GOPROCONTROL_MAVLINK_CAMERA_CALLBACK_H

typedef enum {
    CMD_SET_MODE
} mavlink_camera_callback_type_t;

typedef struct {
    int mode;
} cmd_set_mode_t;

typedef struct {
    mavlink_camera_callback_type_t type;
    union {
        cmd_set_mode_t cmd_set_mode;
    };
} mavlink_camera_callback_t;

typedef int32_t mavlink_camera_err_t;
#define CAMERA_OK 0
#define CAMERA_FAIL 1

typedef mavlink_camera_err_t (*mavlink_camera_callback_handler_t)(mavlink_camera_callback_t callback, void *pvParameters);

#endif //GOPROCONTROL_MAVLINK_CAMERA_CALLBACK_H
