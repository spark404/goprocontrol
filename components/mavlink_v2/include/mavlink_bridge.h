//
// Created by Hugo Trippaers on 03/01/2021.
//

#ifndef GOPROCONTROL_MAVLINK_BRIDGE_H
#define GOPROCONTROL_MAVLINK_BRIDGE_H

__BEGIN_DECLS

extern mavlink_system_t mavlink_system;

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#define MAVLINK_START_UART_SEND mavlink_start_uart_send
#define MAVLINK_END_UART_SEND mavlink_end_uart_send
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

void mavlink_start_uart_send(mavlink_channel_t chan, int length);
void mavlink_end_uart_send(mavlink_channel_t chan, int length);
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);

#ifdef I
  #warning I is defined, undefining to avoid issue with the ardupilot PID message
  #undef I
#endif

#include <ardupilotmega/mavlink.h>

__END_DECLS

#endif //GOPROCONTROL_MAVLINK_BRIDGE_H
