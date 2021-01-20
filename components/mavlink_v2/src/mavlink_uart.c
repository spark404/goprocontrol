#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "mavlink_uart.h"

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#if defined CONFIG_MAVLINK_UART_0
    #define MAVLINK_UART 0
#elif defined CONFIG_MAVLINK_UART_1
    #define MAVLINK_UART 1
#elif defined CONFIG_MAVLINK_UART_2
    #define MAVLINK_UART 2
#endif

#ifdef CONFIG_MAVLINK_UART_CUSTOM_PINS
    #define MAVLINK_UART_TXD  (CONFIG_MAVLINK_UART_TX_PIN)
    #define MAVLINK_UART_RXD  (CONFIG_MAVLINK_UART_RX_PIN)
#else
    #define MAVLINK_UART_TXD  (UART_PIN_NO_CHANGE)
    #define MAVLINK_UART_RXD  (UART_PIN_NO_CHANGE)
#endif
#define MAVLINK_UART_RTS  (UART_PIN_NO_CHANGE)
#define MAVLINK_UART_CTS  (UART_PIN_NO_CHANGE)

#define ESP_MEM_CHECK(TAG, a, action) if (!(a)) {                                             \
        ESP_LOGE(TAG,"%s:%d (%s): %s", __FILE__, __LINE__, __FUNCTION__, "Memory exhausted"); \
        action;                                                                               \
        }

static const char *TAG = "mavlink_uart";
static QueueHandle_t uart_queue;
static SemaphoreHandle_t uart_write;
static int mavlink_uart_num = -1;

static void mavlink_uart_handler_task(void *pvParameters);

mavlink_uart_handle_t mavlink_uart_configure()
{
    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
    };

    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(MAVLINK_UART, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(MAVLINK_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MAVLINK_UART, MAVLINK_UART_TXD, MAVLINK_UART_RXD, MAVLINK_UART_RTS, MAVLINK_UART_CTS));

    mavlink_uart_handle_t handle = calloc(1, sizeof(mavlink_uart_handle_t));
    ESP_MEM_CHECK(TAG, handle, return NULL);
    handle->uart_num = MAVLINK_UART;
    handle->incoming_message_queue = xQueueCreate(20, sizeof(mavlink_message_t));

    // Workaround for the mavlink_uart_send_bytes function
    mavlink_uart_num = MAVLINK_UART;

    if (handle->incoming_message_queue == NULL) {
        ESP_LOGE(TAG, "Failed to initialize the incoming message queue");
        mavlink_uart_destroy(handle);
        return NULL;
    }

    uart_write = xSemaphoreCreateBinary();
    if (uart_write == NULL) {
        ESP_LOGE(TAG, "Failed to initialize the write semaphore");
        mavlink_uart_destroy(handle);
        return NULL;
    }
    xSemaphoreGive(uart_write);

    return handle;
}

esp_err_t mavlink_uart_connect(mavlink_uart_handle_t mavlink_uart_handle)
{
    if (mavlink_uart_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xTaskCreate(mavlink_uart_handler_task, "mavlink_uart_handler_task", 4096, mavlink_uart_handle, 12, &(mavlink_uart_handle->uart_handler_task));

    return ESP_OK;
}

esp_err_t mavlink_uart_destroy(mavlink_uart_handle_t mavlink_uart_handle)
{
    if (mavlink_uart_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mavlink_uart_handle->uart_handler_task != NULL) {
        vTaskDelete(mavlink_uart_handle->uart_handler_task);
    }

    if (uart_is_driver_installed(mavlink_uart_handle->uart_num) == 0) {
        ESP_ERROR_CHECK(uart_disable_rx_intr(mavlink_uart_handle->uart_num));
        ESP_ERROR_CHECK(uart_driver_delete(mavlink_uart_handle->uart_num));
    }

    return ESP_OK;
}

_Noreturn static void mavlink_uart_handler_task(void *pvParameters)
{
    mavlink_uart_handle_t handle = (mavlink_uart_handle_t)pvParameters;

    mavlink_status_t status;
    mavlink_message_t msg;

    uart_event_t event;
    // size_t buffered_size;
    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.

        if (xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type) {
                /* We'd better handler data event fast, there would be much more data events than
                 * other types of events. If we take too much time on data event, the queue might
                 * be full.*/
                case UART_DATA:
                    uart_read_bytes(handle->uart_num, dtmp, event.size, portMAX_DELAY);
                    for (int i = 0; i < event.size; i++) {
                        if (mavlink_frame_char(MAVLINK_COMM_0, *(dtmp + i), &msg, &status) != MAVLINK_FRAMING_INCOMPLETE) {
                            // Make a copy of the message and send it on the queue
                            if (xQueueSend(handle->incoming_message_queue, &msg, 0) != pdPASS) {
                                ESP_LOGE(TAG, "Unable to send message to handler queue");
                            }
                        }
                    }

                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }

    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void mavlink_uart_send_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
    if (mavlink_uart_num == -1) {
        ESP_LOGE(TAG, "[UART ERROR] No UART configured");
        return;
    }

    if (xSemaphoreTake(uart_write, 500 / portTICK_PERIOD_MS) != pdPASS) {
        ESP_LOGE(TAG, "[UART ERROR] Timeout on uart_write semaphore");
    }

    int bytes_written = uart_write_bytes(mavlink_uart_num, (const char *)ch, length);
    if (bytes_written != length) {
        ESP_LOGE(TAG, "[UART ERROR]: expected to write %d, but wrote %d", length, bytes_written);
    }

    xSemaphoreGive(uart_write);
}

void mavlink_uart_start_send(mavlink_channel_t chan, int length)
{
    // Left blank
}

void mavlink_uart_end_send(mavlink_channel_t chan, int length)
{
    // Left blank
}


