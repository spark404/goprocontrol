//
// Created by Hugo Trippaers on 03/01/2021.
//

#ifndef GOPROCONTROL_GOPRO_H
#define GOPROCONTROL_GOPRO_H

#include <freertos/task.h>
#include <esp_http_client.h>
#include "cJSON.h"

typedef enum {
    GOPRO_WIFI_DISCONNECTED,
    GOPRO_WIFI_CONNECTED,
    GOPRO_HTTP_CONNECTED,
    GOPRO_HTTP_DISCONNECTED
} gopro_connection_state_t;

typedef struct {
    gopro_connection_state_t connection_state;
    char *camera_name;
    char *camera_mac;

    // Internal stuff
    TaskHandle_t manager_task;
} gopro_connection_t;

typedef struct {
    int ready;
    char *camera_name;
} gopro_status_t;

esp_err_t gopro_init(gopro_connection_t *gopro_connection);

typedef enum {
    GOPRO_STILL,
    GOPRO_VIDEO
} gopro_mode_t;

esp_err_t gopro_setmode(gopro_connection_t *gopro_connection, gopro_mode_t mode);

#endif //GOPROCONTROL_GOPRO_H
