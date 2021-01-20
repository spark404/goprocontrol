#include <alloca.h>
//
// Created by Hugo Trippaers on 12/09/2020.
//

#ifndef POLEOS_BLINK_H
#define POLEOS_BLINK_H

typedef enum {
    BLINK_STATUS_INITIALIZING,
    BLINK_STATUS_READY,
    BLINK_STATUS_ERROR,
} blink_status_t;

typedef blink_status_t (*status_callback_t)();
typedef struct {
    int default_delay_in_ms;
    status_callback_t status_cb;
} blink_config_t;

_Noreturn void blink_task(void *);

#endif //POLEOS_BLINK_H
