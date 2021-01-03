#include <ctype.h>
#include <complex.h>
/**
 * Global define for the Mavlink subsystem
 */

#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "mavlink_v2.h"

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define MAVLINK_UART UART_NUM_2
#define MAVLINK_UART_TXD  (GPIO_NUM_4)
#define MAVLINK_UART_RXD  (GPIO_NUM_5)
#define MAVLINK_UART_RTS  (UART_PIN_NO_CHANGE)
#define MAVLINK_UART_CTS  (UART_PIN_NO_CHANGE)

static const char *TAG = "mavlink_uart";
static QueueHandle_t uart0_queue;
static QueueHandle_t mavlink_incoming_queue;

#include "mavlink_types.h"
mavlink_system_t mavlink_system = {
        CONFIG_MAVLINK_SYSTEM_ID, // System ID (1-255)
        CONFIG_MAVLINK_COMPONENT_ID  // Component ID (a MAV_COMPONENT value)
};

#include "mavlink_bridge.h"

esp_err_t configure_uart() {
    const int uart_num = MAVLINK_UART;
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

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE *2, 20, &uart0_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, MAVLINK_UART_TXD, MAVLINK_UART_RXD, MAVLINK_UART_RTS, MAVLINK_UART_CTS));

    return ESP_OK;
}

_Noreturn static void uart_event_task(void *pvParameters)
{
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = 0;

    uart_event_t event;
    // size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.

        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch(event.type) {
                /* We'd better handler data event fast, there would be much more data events than
                 * other types of events. If we take too much time on data event, the queue might
                 * be full.*/
                case UART_DATA:
                    uart_read_bytes(MAVLINK_UART, dtmp, event.size, portMAX_DELAY);

                    for (int i =0; i< event.size; i++) {
                        if (mavlink_frame_char(chan, *(dtmp+i), &msg, &status) != MAVLINK_FRAMING_INCOMPLETE) {
                            // Make a copy of the message and send it on the queue
                            if (xQueueSend(mavlink_incoming_queue, &msg, 0) != pdPASS) {
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

_Noreturn static void mavlink_heartbeat_task(void * pvParameters) {
    for (;;) {
        mavlink_msg_heartbeat_send(0, MAV_TYPE_CAMERA, 0, 0, 0, MAV_STATE_ACTIVE);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

_Noreturn static void mavlink_message_handler(void *pvParameters) {
    mavlink_message_t msg;

    for (;;) {
        if (xQueueReceive(mavlink_incoming_queue, (void *) &msg, (portTickType) portMAX_DELAY)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    // Hearthbeat, do nothing for now
                    break;
                case MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    // Nothing but sending COMMAND_ACK
                    mavlink_msg_command_ack_send(0, MAV_CMD_REQUEST_CAMERA_INFORMATION, MAV_CMD_ACK_OK, 255, 0, msg.sysid, msg.compid);
                    ESP_LOGD(TAG, "Ack request for camera information from component %d of system %d", msg.compid, msg.sysid);
                    break;
                //  These messages can be safely ignored
                case MAVLINK_MSG_ID_PARAM_VALUE:
                case MAVLINK_MSG_ID_TIMESYNC:
                case MAVLINK_MSG_ID_STATUSTEXT:
                    break;
                default:
                    ESP_LOGD(TAG, "Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid,
                             msg.seq, msg.compid, msg.sysid);
                    break;
            }
        }
    }
}

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length) {
    int bytes_written = uart_write_bytes(MAVLINK_UART, (const char *)ch, length);
    if (bytes_written != length) {
        ESP_LOGE(TAG, "[UART ERROR]: expected to write %d, but wrote %d", length, bytes_written);
    }
}

void mavlink_start_uart_send(mavlink_channel_t chan, int length) {

}

void mavlink_end_uart_send(mavlink_channel_t chan, int length) {

}

esp_err_t enable_mavlink_on_uart() {
    if (configure_uart() != ESP_OK) {
        return ESP_FAIL;
    }

    mavlink_incoming_queue = xQueueCreate(20, sizeof(mavlink_message_t));
    if (mavlink_incoming_queue == NULL) {
        ESP_LOGE(TAG, "Failed to initialize the incoming message queue");
        return ESP_FAIL;
    }

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    xTaskCreate(mavlink_heartbeat_task, "mavlink_heartbeat_task", 4096, NULL, 11, NULL);
    xTaskCreate(mavlink_message_handler, "mavlink_message_handler", 4096, NULL, 10, NULL);

    return ESP_OK;
}

esp_err_t disable_mavlink_on_uart() {
    if (uart_is_driver_installed(MAVLINK_UART) == 0) {
        ESP_ERROR_CHECK(uart_disable_rx_intr(MAVLINK_UART));
        ESP_ERROR_CHECK(uart_driver_delete(MAVLINK_UART));
    }

    return ESP_OK;
}

