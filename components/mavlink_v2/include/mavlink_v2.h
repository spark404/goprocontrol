//
// Created by Hugo Trippaers on 03/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_V2_H
#define GOPROCONTROL_MAVLINK_V2_H

#include "mavlink_camera_callback.h"

esp_err_t mavlink_v2_init(mavlink_camera_callback_handler_t mavlink_camera_callback);
esp_err_t enable_mavlink_on_uart();

#endif //GOPROCONTROL_MAVLINK_V2_H
