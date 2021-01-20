#include <alloca.h>
/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "blink.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_STATUS_LED_GPIO

#define TAG "blink_task"

_Noreturn void blink_task(void *pvParameters)
{
    assert(pvParameters != NULL);

    blink_config_t *config = (blink_config_t *)pvParameters;

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    int delay_in_ms = config->default_delay_in_ms;
    int total_delay = 0;

    while (1) {
        if (total_delay >= 1000) {
            switch (config->status_cb()) {
                case BLINK_STATUS_INITIALIZING:
                    delay_in_ms = 500;
                    break;
                case BLINK_STATUS_READY:
                    delay_in_ms = 0;
                    break;
                case BLINK_STATUS_ERROR:
                    delay_in_ms = 250;
                    break;
                default:
                    delay_in_ms = config->default_delay_in_ms;
            }
            total_delay = 0;
        }

        if (delay_in_ms == 0) {
            // always on
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            total_delay += 1000;
        } else {
            /* Blink off (output low) */
            gpio_set_level(BLINK_GPIO, 0);
            vTaskDelay(delay_in_ms / portTICK_PERIOD_MS);

            /* Blink on (output high) */
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(delay_in_ms / portTICK_PERIOD_MS);

            total_delay += 2 * delay_in_ms;
        }
    }
}
