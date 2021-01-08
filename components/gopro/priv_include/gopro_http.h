//
// Created by Hugo Trippaers on 04/01/2021.
//

#ifndef GOPROCONTROL_GOPRO_HTTP_C_H
#define GOPROCONTROL_GOPRO_HTTP_C_H

#include "gopro_internal.h"

esp_err_t gopro_http_init(gopro_callback_handler_t handler, void *user_data);

esp_err_t gopro_http_get_info();
esp_err_t gopro_http_get_status(gopro_internal_status_t *);
esp_err_t gopro_http_set_mode(int);
esp_err_t gopro_http_shutter(int);

#endif //GOPROCONTROL_GOPRO_HTTP_C_H
