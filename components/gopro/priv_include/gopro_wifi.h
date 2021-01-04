//
// Created by Hugo Trippaers on 04/01/2021.
//

#ifndef GOPROCONTROL_GOPRO_WIFI_H
#define GOPROCONTROL_GOPRO_WIFI_H

#include "gopro_internal.h"

esp_err_t gopro_wifi_init(gopro_callback_handler_t handler, void *user_data);
esp_err_t gopro_wifi_connect(void);

#endif //GOPROCONTROL_GOPRO_WIFI_H
