//
// Created by Hugo Trippaers on 03/01/2021.
//

#include <esp_err.h>
#include <esp_log.h>
#include <esp_http_client.h>

static const char *TAG = "gopro";

static esp_http_client_handle_t client;

esp_err_t _http_event_handle(esp_http_client_event_t *evt);

esp_err_t gopro_init() {
    ESP_LOGI(TAG, "Initializing GoPro Connection");
    esp_http_client_config_t config = {
            .url = "http://10.5.5.9/gp/gpControl/info",
            .method = HTTP_METHOD_GET,
            .timeout_ms = 20 * 1000,
            .event_handler = _http_event_handle,
    };

    client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to create http client for GoPro");
        return ESP_FAIL;
    }

    esp_err_t result = esp_http_client_perform(client);
    if (result != ESP_OK) {
        esp_http_client_cleanup(client);
        return result;
    }

    ESP_LOGI(TAG, "Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
    return ESP_OK;
}

esp_err_t gopro_deinit() {
    ESP_LOGI(TAG, "Destroying GoPro Connection");
    esp_http_client_cleanup(client);

    return ESP_OK;
}

esp_err_t gopro_get_status() {
    esp_http_client_set_url(client, "http://10.5.5.9/gp/gpControl/status");
    esp_http_client_set_method(client, HTTP_METHOD_GET);

    esp_err_t result = esp_http_client_perform(client);
    if (result != ESP_OK) {
        return result;
    }

    ESP_LOGI(TAG, "Status = %d, content_length = %d",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
    return ESP_OK;
}


void gopro_wakeup() {
    // Send a WOL

}

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
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
                printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

