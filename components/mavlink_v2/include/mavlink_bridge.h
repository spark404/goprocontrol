//
// Created by Hugo Trippaers on 03/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_BRIDGE_H
#define GOPROCONTROL_MAVLINK_BRIDGE_H

#include "mavlink_types.h"

__BEGIN_DECLS

extern mavlink_system_t mavlink_system;

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#define MAVLINK_START_UART_SEND mavlink_uart_start_send
#define MAVLINK_END_UART_SEND mavlink_uart_end_send
#define MAVLINK_SEND_UART_BYTES mavlink_uart_send_bytes

void mavlink_uart_start_send(mavlink_channel_t chan, int length);
void mavlink_uart_end_send(mavlink_channel_t chan, int length);
void mavlink_uart_send_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);

#include <ardupilotmega/mavlink.h>

__END_DECLS

#endif //GOPROCONTROL_MAVLINK_BRIDGE_H
