//
// Created by Hugo Trippaers on 04/01/2021.
//
#include <string.h>

#include <esp_log.h>
#include <esp_http_client.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "cJSON.h"

#include "gopro_internal.h"

#define GOPRO_BASEURL "http://10.5.5.9"

static const char *TAG = "gopro_http";

typedef struct {
    char *buffer;
    size_t buffer_size;
} response_buffer_t;

static response_buffer_t response_buffer;
static esp_http_client_handle_t client;
static SemaphoreHandle_t http_semaphore;

static esp_err_t gopro_http_request(char *uri);
static esp_err_t gopro_http_request_with_json(char *uri, cJSON **);
static esp_err_t http_event_handle(esp_http_client_event_t *evt);


esp_err_t gopro_http_init() {
    response_buffer.buffer = malloc(2048);
    response_buffer.buffer_size = 2048;

    if (response_buffer.buffer == NULL) {
        ESP_LOGE(TAG, "Unable to allocate space for return buffer");
        return ESP_FAIL;
    }

    http_semaphore = xSemaphoreCreateBinary();
    if (http_semaphore == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateBinary failed");
        return ESP_FAIL;
    }
    xSemaphoreGive(http_semaphore);

    esp_http_client_config_t config = {
            .url = GOPRO_BASEURL,
            .method = HTTP_METHOD_GET,
            .timeout_ms = 5 * 1000,
            .event_handler = http_event_handle,
            .user_data = &response_buffer
    };
    client = esp_http_client_init(&config);

    return ESP_OK;
}


esp_err_t gopro_http_cleanup() {
    xSemaphoreTake(http_semaphore, 10000 / portTICK_PERIOD_MS);
    vSemaphoreDelete(http_semaphore);

    esp_http_client_cleanup(client);
    free(response_buffer.buffer);

    return ESP_OK;
}


esp_err_t gopro_http_get_info(gopro_info_t *gopro_info) {
    cJSON *json = NULL;
    if (gopro_http_request_with_json("/gp/gpControl/info", &json) != ESP_OK || json == NULL) {
        return ESP_FAIL;
    }

    cJSON *info = cJSON_GetObjectItemCaseSensitive(json, "info");
    if (cJSON_IsObject(info))
    {
        cJSON *model_name = cJSON_GetObjectItemCaseSensitive(info, "model_name");
        if (cJSON_IsString(model_name) && model_name->valuestring != NULL) {
            gopro_info->model_name = malloc(strlen(model_name->valuestring) + 1);
            bcopy(model_name->valuestring, gopro_info->model_name, strlen(model_name->valuestring) + 1);
        }

        cJSON *ap_mac = cJSON_GetObjectItemCaseSensitive(info, "ap_mac");
        if (cJSON_IsString(ap_mac) && ap_mac->valuestring != NULL) {
            gopro_info->mac_address = malloc(strlen(ap_mac->valuestring) + 1);
            bcopy(ap_mac->valuestring, gopro_info->mac_address, strlen(ap_mac->valuestring) + 1);
        }
    }

    cJSON_Delete(json);

    return ESP_OK;
}

#define STATUS_CURRENT_MODE "43"
#define STATUS_CURRENT_RECORDING_VIDEO_DURATION "13"
#define STATUS_NUM_ALL_PHOTOS_TAKEN "38"
#define STATUS_NUM_ALL_VIDEOS_TAKEN "39"
#define STATUS_REMAINING_FREE_BYTES "54"
#define STATUS_PROCESSING_STATUS "8"

static int64_t last_update = 0;
static cJSON *gopro_status = NULL;

esp_err_t gopro_http_get_status(gopro_internal_status_t *gopro_internal_status) {
    if (esp_timer_get_time() - last_update > 500000 || gopro_status == NULL) {
        ESP_LOGD(TAG, "Retrieving status object from device");
        if (gopro_status != NULL) {
            cJSON_Delete(gopro_status);
            gopro_status = NULL;
        }

        if (gopro_http_request_with_json("/gp/gpControl/status", &gopro_status) != ESP_OK || gopro_status == NULL) {
            ESP_LOGW(TAG, "Failed to retrieve gopro status");
            return ESP_FAIL;
        }

        last_update = esp_timer_get_time();
    } else {
        ESP_LOGD(TAG, "Retrieving gopro status object from cache");
    }

    if (gopro_internal_status == NULL) {
        return ESP_OK;
    }

    cJSON *status_object = cJSON_GetObjectItem(gopro_status, "status");
    if (status_object == NULL || !cJSON_IsObject(status_object)) {
        cJSON_Delete(gopro_status);
        gopro_status = NULL;
        return ESP_FAIL;
    }

    cJSON *field;

    field = cJSON_GetObjectItem(status_object, STATUS_CURRENT_MODE);
    if (field != NULL && cJSON_IsNumber(field)) {
        gopro_internal_status->current_mode = field->valueint;
    }

    field = cJSON_GetObjectItem(status_object, STATUS_CURRENT_RECORDING_VIDEO_DURATION);
    if (field != NULL && cJSON_IsNumber(field)) {
        gopro_internal_status->recording_time_ms = field->valueint;
    }

    field = cJSON_GetObjectItem(status_object, STATUS_NUM_ALL_PHOTOS_TAKEN);
    if (field != NULL && cJSON_IsNumber(field)) {
        gopro_internal_status->num_photos_taken = field->valueint;
    }

    field = cJSON_GetObjectItem(status_object, STATUS_NUM_ALL_VIDEOS_TAKEN);
    if (field != NULL && cJSON_IsNumber(field)) {
        gopro_internal_status->num_videos_taken = field->valueint;
    }

    field = cJSON_GetObjectItem(status_object, STATUS_REMAINING_FREE_BYTES);
    if (field != NULL && cJSON_IsNumber(field)) {
        gopro_internal_status->remaining_free_bytes = field->valueint;
    }

    field = cJSON_GetObjectItem(status_object, STATUS_PROCESSING_STATUS);
    if (field != NULL && cJSON_IsNumber(field)) {
        gopro_internal_status->processing_flag = field->valueint;
    }

    return ESP_OK;
}


esp_err_t gopro_http_set_mode(int mode) {
    if (mode < 0 || mode > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    char *modebase = "/gp/gpControl/command/mode?p=";
    char url[strlen(modebase) + 2];
    strcpy(url, modebase);
    itoa(mode, url+strlen(url), 10);

    return gopro_http_request(url);
}


esp_err_t gopro_http_shutter(int enable) {
    char *shutterbase = "/gp/gpControl/command/shutter?p=";
    char url[strlen(shutterbase) + 2];
    strcpy(url, shutterbase);
    itoa(enable ? 1 : 0, url+strlen(url), 10);

    return gopro_http_request(url);
}


static esp_err_t gopro_http_request_with_json(char *uri, cJSON **response_data) {
    cJSON *json = NULL;
    esp_err_t result = ESP_FAIL;

    if (uri == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(http_semaphore, 1500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take http_semaphore");
        return ESP_ERR_TIMEOUT;
    };

    char url[strlen(GOPRO_BASEURL) + strlen(uri) + 1];
    strcpy(url, GOPRO_BASEURL);
    strcat(url, uri);

    esp_http_client_set_url(client, url);

    if (esp_http_client_perform(client) != ESP_OK) {
        ESP_LOGW(TAG, "request failed: esp_http_client_perform failed");
        goto error;
    }

    int response_code = esp_http_client_get_status_code(client);
    if (response_code != 200) {
        ESP_LOGW(TAG, "request failed: unexpected response code %d with buffer size %d", response_code, response_buffer.buffer_size);
        goto error;
    }

    // HTTP_EVENT_ON_DATA handler ensures the buffer is NULL terminated
    json = cJSON_Parse(response_buffer.buffer);
    if (json == NULL) {
        ESP_LOGW(TAG, "request failed: unable to parse JSON");
        ESP_LOG_BUFFER_HEXDUMP(TAG, response_buffer.buffer, response_buffer.buffer_size, ESP_LOG_DEBUG);
        goto error;
    }

    *response_data = json;
    result = ESP_OK;

    error:
    xSemaphoreGive(http_semaphore);
    return result;
}


static esp_err_t gopro_http_request(char *uri) {
    if (uri == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(http_semaphore, 1500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take http_semaphore");
        return ESP_ERR_TIMEOUT;
    };

    char url[strlen(GOPRO_BASEURL) + strlen(uri) + 1];
    strcpy(url, GOPRO_BASEURL);
    strcat(url, uri);
    esp_http_client_set_url(client, url);

    if (esp_http_client_perform(client) != ESP_OK) {
        xSemaphoreGive(http_semaphore);
        return ESP_FAIL;
    }

    esp_err_t result = esp_http_client_get_status_code(client) == 200 ? ESP_OK : ESP_FAIL;

    xSemaphoreGive(http_semaphore);
    return result;
}


static esp_err_t http_event_handle(esp_http_client_event_t *evt)
{
    static char *output_buffer;         // Buffer to store response of http request from event handler
    static int output_len;              // Stores number of bytes read
    static size_t output_buffer_size;   // Stores the size of the malloc-ed output_buffer

    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGW(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_DATA:
            // ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (evt->user_data == NULL) {
                // No need to capture data
                return ESP_OK;
            }

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
            break;
        case HTTP_EVENT_ON_FINISH:
            // ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (evt->user_data == NULL || output_buffer == NULL) {
                // No need to capture data
                return ESP_OK;
            }

            response_buffer_t *data = evt->user_data;
            if (output_len > data->buffer_size) {
                ESP_LOGW(TAG, "Not enough space in result buffer");
            }
            bcopy(output_buffer, data->buffer,
                  output_len <= data->buffer_size ? output_len : data->buffer_size);

            free(output_buffer);
            output_buffer = NULL;
            output_len = 0;
            break;
        default:
            // do nothing
            break;
    }
    return ESP_OK;
}

