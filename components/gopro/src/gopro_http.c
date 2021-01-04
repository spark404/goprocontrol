//
// Created by Hugo Trippaers on 04/01/2021.
//
#include <string.h>

#include <esp_log.h>
#include <esp_http_client.h>
#include "cJSON.h"

#include "gopro_internal.h"

static const char *TAG = "gopro_http";

typedef struct {
    gopro_callback_handler_t handler;
    void *user_data;
} callback_t;
static callback_t callback;

typedef struct {
    char *buffer;
    size_t buffer_size;
} response_data_t;

esp_err_t http_event_handle(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    static size_t output_buffer_size;      // Stores the size of the malloc-ed output_buffer
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (output_buffer == NULL) {
                    // If we don't know the content size we alloc a guestimate
                    if (esp_http_client_get_content_length(evt->client) == -1) {
                        output_buffer_size = 2048;
                    } else {
                        output_buffer_size = esp_http_client_get_content_length(evt->client) + 1;
                    }
                    output_buffer = (char *) malloc(output_buffer_size);

                    if (output_buffer == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate %d bytes memory for output buffer", output_buffer_size);
                        return ESP_FAIL;
                    }

                    bzero(output_buffer, output_buffer_size);
                    output_len = 0;
                }

                if (output_len + evt->data_len + 1 > output_buffer_size) {
                    ESP_LOGE(TAG, "Output buffer overrun: allocated %d but needs at least %d", output_buffer_size, output_len + evt->data_len);
                    return ESP_FAIL;
                }

                memcpy(output_buffer + output_len, evt->data, evt->data_len);
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                if (evt->user_data != NULL) {
                    response_data_t *data = evt->user_data;
                    data->buffer_size = output_len;
                    data->buffer = malloc(output_len);
                    bcopy(output_buffer, data->buffer, output_len);
                }

                free(output_buffer);
                output_buffer = NULL;
                output_len = 0;
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

esp_err_t gopro_http_init(gopro_callback_handler_t handler, void *user_data) {
    callback.handler = handler;
    callback.user_data = user_data;

    return ESP_OK;
}

esp_err_t gopro_http_connect() {
    response_data_t response;
    esp_http_client_config_t config = {
            .url = "http://10.5.5.9/gp/gpControl/info",
            .method = HTTP_METHOD_GET,
            .timeout_ms = 20 * 1000,
            .event_handler = http_event_handle,
            .user_data = &response
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to create http client for GoPro");
        return ESP_FAIL;
    }

    if (esp_http_client_perform(client) != ESP_OK) {
        goto error;
    }

    int response_code = esp_http_client_get_status_code(client);
    if (response_code != 200 || response.buffer == NULL) {
        goto error;
    }

    // HTTP_EVENT_ON_DATA handler ensures the buffer is NULL terminated
    cJSON *json = cJSON_Parse(response.buffer);
    if (json == NULL) {
        goto error;
    }

    gopro_callback_t gopro_callback = {
            .type = INFO_RECEIVED
    };

    cJSON *info = cJSON_GetObjectItemCaseSensitive(json, "info");
    if (cJSON_IsObject(info))
    {
        cJSON *model_name = cJSON_GetObjectItemCaseSensitive(info, "model_name");
        if (cJSON_IsString(model_name) && model_name->valuestring != NULL) {
            gopro_callback.gopro_info.model_name = malloc(strlen(model_name->valuestring) + 1);
            bcopy(model_name->valuestring, gopro_callback.gopro_info.model_name, strlen(model_name->valuestring) + 1);
        }

        cJSON *ap_mac = cJSON_GetObjectItemCaseSensitive(info, "ap_mac");
        if (cJSON_IsString(ap_mac) && ap_mac->valuestring != NULL) {
            gopro_callback.gopro_info.mac_address = malloc(strlen(ap_mac->valuestring) + 1);
            bcopy(ap_mac->valuestring, gopro_callback.gopro_info.mac_address, strlen(ap_mac->valuestring) + 1);
        }
    }

    if (callback.handler) {
        callback.handler(gopro_callback, callback.user_data);
    }

    esp_err_t result = esp_http_client_get_status_code(client) == 200 ? ESP_OK : ESP_FAIL;
    esp_http_client_cleanup(client);
    return result;

    error:
    esp_http_client_cleanup(client);
    return ESP_FAIL;
}

esp_err_t gopro_http_set_mode(int mode) {
    if (mode < 0 || mode > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[46];
    strcpy(url, "http://10.5.5.9/gp/gpControl/command/mode?p=");
    char mode_id[2];
    itoa(mode, mode_id, 10);
    strcat(url, mode_id);

    esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_GET,
            .timeout_ms = 5 * 1000,
            .event_handler = http_event_handle,
            .user_data = NULL
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (esp_http_client_perform(client) != ESP_OK) {
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_err_t result = esp_http_client_get_status_code(client) == 200 ? ESP_OK : ESP_FAIL;
    esp_http_client_cleanup(client);
    return result;
}

esp_err_t gopro_http_get_status() {
    esp_http_client_config_t config = {
            .url = "http://10.5.5.9/gp/gpControl/status",
            .method = HTTP_METHOD_GET,
            .timeout_ms = 20 * 1000,
            .event_handler = http_event_handle,
            .user_data = NULL
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (esp_http_client_perform(client) != ESP_OK) {
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_err_t result = esp_http_client_get_status_code(client) == 200 ? ESP_OK : ESP_FAIL;
    esp_http_client_cleanup(client);
    return result;
}

