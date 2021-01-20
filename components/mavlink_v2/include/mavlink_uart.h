//
// Created by Hugo Trippaers on 19/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_UART_H
#define GOPROCONTROL_MAVLINK_UART_H

#include "mavlink_bridge.h"

typedef struct {
    int uart_num;
    QueueHandle_t incoming_message_queue;
    TaskHandle_t uart_handler_task;
} *mavlink_uart_handle_t;

mavlink_uart_handle_t mavlink_uart_configure();
esp_err_t mavlink_uart_connect(mavlink_uart_handle_t mavlink_uart_handle);
esp_err_t mavlink_uart_destroy(mavlink_uart_handle_t mavlink_uart_handle);

#endif //GOPROCONTROL_MAVLINK_UART_H
