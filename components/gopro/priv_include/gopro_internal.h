//
// Created by Hugo Trippaers on 04/01/2021.
//

#ifndef GOPROCONTROL_GOPRO_INTERNAL_H
#define GOPROCONTROL_GOPRO_INTERNAL_H

// Set via Kconfig
#define GOPRO_ESP_WIFI_SSID      CONFIG_GOPRO_WIFI_SSID
#define GOPRO_ESP_WIFI_PASS      CONFIG_GOPRO_WIFI_PASSWORD
#define GOPRO_RDP_MAC_ADDRESS    CONFIG_GOPRO_MAC_ADDRESS

typedef enum {
    WIFI_DISCONNECTED,
    INFO_RECEIVED
} gopro_callback_type_t;

typedef struct {
    char *model_name;
    char *mac_address;
} gopro_info_t;

typedef struct {
    int processing_flag;      //  8
    int recording_time_ms;    // 13
    int num_photos_taken;     // 38
    int num_videos_taken;     // 39
    int current_mode;         // 43
    int remaining_free_bytes; // 54
} gopro_internal_status_t;

typedef struct {
    gopro_callback_type_t type;
    union {
        gopro_info_t gopro_info;
    };
} gopro_callback_t;

typedef void (*gopro_callback_handler_t)(gopro_callback_t callback, void *pvParameters);



#endif //GOPROCONTROL_GOPRO_INTERNAL_H
