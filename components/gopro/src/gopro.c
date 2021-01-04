//
// Created by Hugo Trippaers on 03/01/2021.
//
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

#include "gopro.h"
#include "gopro_wifi.h"
#include "gopro_http.h"
#include "gopro_wol.h"

static const char *TAG = "gopro";

/* This task runs approximately every 2.5 seconds and should
 * monitor the connection to the gopro and keep it alive
 */
_Noreturn void connection_manager_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting connection manager task for GoPro");
    gopro_connection_t *connection = pvParameters;
    unsigned long last_time = 0;
    for(;;) {
        unsigned long current_time = esp_timer_get_time() / 1000;
        switch (connection->connection_state) {
            case GOPRO_WIFI_DISCONNECTED:
                if (gopro_wifi_connect() != ESP_OK) {
                    break;
                }
                ESP_LOGD(TAG, "WiFi connected, state set to GOPRO_WIFI_CONNECTED");
                connection->connection_state = GOPRO_WIFI_CONNECTED;
                // Fallthrough to the next step
            case GOPRO_WIFI_CONNECTED:
            case GOPRO_HTTP_DISCONNECTED:
                if (gopro_http_connect() != ESP_OK) {
                    if (connection->camera_mac != NULL) {
                        gopro_wol_send(connection->camera_mac);
                    } else {
                        ESP_LOGW(TAG, "No mac address set, unable to send WOL");
                    }
                    break;
                }
                ESP_LOGD(TAG, "HTTP info retrieved, state set to GOPRO_WIFI_CONNECTED");
                connection->connection_state = GOPRO_HTTP_CONNECTED;
                break;
            case GOPRO_HTTP_CONNECTED:
                if (gopro_keepalive_send() != ESP_OK) {
                    ESP_LOGD(TAG, "Keepalive send failed, state set to GOPRO_HTTP_DISCONNECTED");
                    connection->connection_state = GOPRO_HTTP_DISCONNECTED;
                }
                break;
        }

        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}

void gopro_callback(gopro_callback_t callback, void *user_data) {
    gopro_connection_t *connection = user_data;

    switch (callback.type) {
        case WIFI_DISCONNECTED:
            connection->connection_state = GOPRO_WIFI_DISCONNECTED;
            break;
        case INFO_RECEIVED:
            ESP_LOGI(TAG, "Received camera details: %s, (mac: %s)", callback.gopro_info.model_name, callback.gopro_info.mac_address);
            connection->camera_name = callback.gopro_info.model_name;
            connection->camera_mac = callback.gopro_info.mac_address;
            break;
        default:
            ESP_LOGI(TAG, "Unhandled GoPro callback of type %d", callback.type);
            break;
    }
}

esp_err_t gopro_init(gopro_connection_t *gopro_connection) {
    if (gopro_connection == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing WiFi");
    gopro_wifi_init(gopro_callback, gopro_connection);
    gopro_http_init(gopro_callback, gopro_connection);

    gopro_connection->connection_state = GOPRO_WIFI_DISCONNECTED;

    /* set the mac address from the configuration
     * will be overwritten by the mac address in the info
     */
    if (strlen(GOPRO_RDP_MAC_ADDRESS) > 0) {
        gopro_connection->camera_mac = GOPRO_RDP_MAC_ADDRESS;
    }

    ESP_LOGI(TAG, "Initializing GoPro Connection");
    xTaskCreate(&connection_manager_task, "connection_manager_task", 10240, gopro_connection, 20, NULL);

    return ESP_OK;
}

esp_err_t gopro_setmode(gopro_connection_t *gopro_connection, gopro_mode_t mode) {
    if (gopro_connection->connection_state != GOPRO_HTTP_CONNECTED) {
        return ESP_FAIL;
    }

    return gopro_http_set_mode(mode == GOPRO_STILL ? 1 : 2);
}

gopro_status_t gopro_get_status(gopro_connection_t *gopro_connection) {
    if (gopro_connection == NULL) {
        gopro_status_t status = {
            .ready = false,
            .camera_name = NULL
        };
        return status;
    }

    gopro_status_t status = {
            .ready = gopro_connection->connection_state == GOPRO_HTTP_CONNECTED,
            .camera_name = gopro_connection->camera_name
    };
    return status;
}





