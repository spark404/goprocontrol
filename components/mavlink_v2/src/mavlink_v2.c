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
#include "mavlink_camera_callback.h"

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define MAVLINK_UART UART_NUM_2
#define MAVLINK_UART_TXD  (GPIO_NUM_4)
#define MAVLINK_UART_RXD  (GPIO_NUM_5)
#define MAVLINK_UART_RTS  (UART_PIN_NO_CHANGE)
#define MAVLINK_UART_CTS  (UART_PIN_NO_CHANGE)

#define MAV_LOG_CMDD(tag, name, cmd) ESP_LOGD(tag, "Received %s (p1 %f, p2 %f, p3 %f, p4 %f, p5 %f, p6 %f, p7 %f", \
    name, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5,cmd.param6, cmd.param7)

static const char *TAG = "mavlink_uart";
static QueueHandle_t uart0_queue;
static QueueHandle_t mavlink_incoming_queue;

static TaskHandle_t heartbeat_task;

esp_err_t mavlink_send_camera_info();
esp_err_t mavlink_send_camera_settings();
esp_err_t mavlink_send_camera_capture_status();
esp_err_t mavlink_camera_start_video_recording();
esp_err_t mavlink_camera_stop_video_recording();
esp_err_t mavlink_camera_send_storage_information(uint8_t storage_id);
esp_err_t mavlink_camera_start_capture();

typedef struct {
    mavlink_camera_callback_handler_t camera_callback;
} mavlink_callbacks_t;
static mavlink_callbacks_t mavlink_callbacks;

#include "mavlink_types.h"
static mavlink_system_t mavlink_system = {
        CONFIG_MAVLINK_SYSTEM_ID, // System ID (1-255)
        CONFIG_MAVLINK_COMPONENT_ID  // Component ID (a MAV_COMPONENT value)
};

#include "mavlink_bridge.h"

/* This will wrap at some point in the future
 * Celebrate when that happens
 */
uint32_t get_time_since_boot_in_ms() {
    int64_t current = esp_timer_get_time();
    return current / 1000;  // ESP Timer is in microseconds
}

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
                        if (mavlink_frame_char(MAVLINK_COMM_0, *(dtmp+i), &msg, &status) != MAVLINK_FRAMING_INCOMPLETE) {
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
    bool first_init_complete = 0;
    for (;;) {
        mavlink_camera_callback_t callback = {
                .type = CMD_GET_STATUS
        };
        cmd_get_status_t cmd_get_status;

        mavlink_camera_err_t result = mavlink_callbacks.camera_callback(callback, &cmd_get_status);
        uint8_t system_status = (result == CAMERA_OK && cmd_get_status.ready) ? MAV_STATE_ACTIVE : MAV_STATE_BOOT;

        if (!first_init_complete && system_status == MAV_STATE_ACTIVE) {
            first_init_complete = true;
        }

        // Hold off on the heartbeats until we have our first initialization done
        if (first_init_complete) {
            mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_CAMERA, MAV_AUTOPILOT_INVALID,
                                       MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0,
                                       system_status);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

_Noreturn static void mavlink_message_handler(void *pvParameters) {
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    for (;;) {
        if (xQueueReceive(mavlink_incoming_queue, (void *) &msg, (portTickType) portMAX_DELAY)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    if (msg.compid == 1 && heartbeat_task == NULL) {
                        ESP_LOGI(TAG, "Received heartbeat from FlightController, activating heartbeat");
                        mavlink_system.sysid = msg.sysid;
                        xTaskCreate(mavlink_heartbeat_task, "mavlink_heartbeat_task", 4096, NULL, 11, &heartbeat_task);
                    }
                    break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    mavlink_msg_command_long_decode(&msg, &cmd);
                    ESP_LOGD(TAG, "Command received: (sysid: %d compid: %d msgid: %d) from compid %d, sysid %d",
                             cmd.target_system, cmd.target_component, cmd.command, msg.compid, msg.sysid);
                    switch (cmd.command) {
                        case MAV_CMD_REQUEST_MESSAGE:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_MESSAGE", cmd);
                            uint32_t requested_message_id = cmd.param1;
                            switch (requested_message_id) {
                                case MAVLINK_MSG_ID_CAMERA_INFORMATION:
                                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                                    mavlink_send_camera_info();
                                    break;
                                case MAVLINK_MSG_ID_CAMERA_SETTINGS:
                                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                                    mavlink_send_camera_settings();
                                    break;
                                case MAVLINK_MSG_ID_STORAGE_INFORMATION:
                                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                                    mavlink_camera_send_storage_information(cmd.param2);
                                    break;
                                case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
                                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                                    mavlink_send_camera_capture_status();
                                    break;
                                default:
                                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_UNSUPPORTED, 255, 0, msg.sysid, msg.compid);
                                    break;
                            }
                            break;
                        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_CAMERA_INFORMATION", cmd);
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                            mavlink_send_camera_info();
                            break;
                        case MAV_CMD_REQUEST_CAMERA_SETTINGS:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_CAMERA_SETTINGS", cmd);
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                            mavlink_send_camera_settings();
                            break;
                        case MAV_CMD_REQUEST_STORAGE_INFORMATION:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_STORAGE_INFORMATION", cmd);
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                            mavlink_camera_send_storage_information(cmd.param2);
                            break;
                        case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS", cmd);
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                            mavlink_send_camera_capture_status();
                            break;
                        case MAV_CMD_SET_CAMERA_MODE:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_SET_CAMERA_MODE", cmd);
                            cmd_set_mode_t set_mode = {
                                    .mode = cmd.param2
                            };
                            mavlink_camera_callback_t callback = {
                                    .type = CMD_SET_MODE,
                                    .cmd_set_mode = set_mode
                            };
                            if (mavlink_callbacks.camera_callback(callback, NULL) != CAMERA_OK) {
                                mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_FAILED, 255, 0, msg.sysid, msg.compid);
                            } else {
                                mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                            }
                            break;
                        case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION", cmd);
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_UNSUPPORTED, 255, 0, msg.sysid, msg.compid);
                            break;
                        case MAV_CMD_IMAGE_START_CAPTURE:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_IMAGE_START_CAPTURE", cmd);
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                            mavlink_camera_start_capture();
                            break;
                        case MAV_CMD_IMAGE_STOP_CAPTURE:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_IMAGE_STOP_CAPTURE", cmd);
                            // No support for intervals yet
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_UNSUPPORTED, 255, 0, msg.sysid, msg.compid);
                            break;
                        case MAV_CMD_VIDEO_START_CAPTURE:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_VIDEO_START_CAPTURE", cmd);
                            if (mavlink_camera_start_video_recording() == ESP_OK) {
                                mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                            } else {
                                mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_FAILED, 0, 0,
                                                             msg.sysid, msg.compid);
                            }
                            break;
                        case MAV_CMD_VIDEO_STOP_CAPTURE:
                            MAV_LOG_CMDD(TAG, "MAV_CMD_VIDEO_STOP_CAPTURE", cmd);
                            if (mavlink_camera_stop_video_recording() == ESP_OK) {
                                mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                            } else {
                                mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_FAILED, 0, 0,
                                                             msg.sysid, msg.compid);
                            }
                            break;
                        default:
                            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_UNSUPPORTED, 255, 0, msg.sysid, msg.compid);
                            break;
                    }
                    break;
                default:
                    //ESP_LOGD(TAG, "Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
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
    // Left blank
}

void mavlink_end_uart_send(mavlink_channel_t chan, int length) {
    // Left blank
}

esp_err_t mavlink_v2_init(mavlink_camera_callback_handler_t mavlink_camera_callback) {
    mavlink_callbacks.camera_callback = mavlink_camera_callback;
    return ESP_OK;
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

esp_err_t mavlink_send_camera_info() {
    mavlink_camera_callback_t callback = {
            .type = CMD_GET_STATUS
    };
    cmd_get_status_t cmd_get_status;

    mavlink_callbacks.camera_callback(callback, &cmd_get_status);

    if (!cmd_get_status.ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t vendor[32];
    uint8_t model[32];
    strcpy((char *) vendor, "ESP32-GOPRO");
    strcpy((char *) model, "HERO 5");

    uint32_t flags =
            CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                    CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                    CAMERA_CAP_FLAGS_HAS_MODES |
                    CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM;

    mavlink_msg_camera_information_send(
            MAVLINK_COMM_0,
            get_time_since_boot_in_ms(),
            vendor,
            model,
            1,
            16.4f,
            1.1f,
            1.1f,
            1920,
            1080,
            0,
            flags,
            1,
            NULL
    );

    return ESP_OK;
}

esp_err_t mavlink_send_camera_settings() {
    mavlink_camera_callback_t callback = {
            .type = CMD_GET_SETTINGS
    };
    cmd_get_settings_t cmd_get_settings;

    if (mavlink_callbacks.camera_callback(callback, &cmd_get_settings) != CAMERA_OK) {
        ESP_LOGW(TAG, "Failed to retrieve settings from the callback");
        return ESP_FAIL;
    }

    uint8_t mode_id = CAMERA_MODE_IMAGE;
    switch (cmd_get_settings.recording_mode) {
        case RECORDING_MODE_STILL:
            mode_id = CAMERA_MODE_IMAGE;
            break;
        case RECORDING_MODE_VIDEO:
            mode_id = CAMERA_MODE_VIDEO;
            break;
    }

    mavlink_msg_camera_settings_send(
            MAVLINK_COMM_0,
            get_time_since_boot_in_ms(),
            mode_id,
            0,
            0
            );

    return ESP_OK;
}

esp_err_t mavlink_send_camera_capture_status() {
    mavlink_camera_callback_t callback = {
            .type = CMD_GET_SETTINGS
    };
    cmd_get_settings_t cmd_get_settings;

    if (mavlink_callbacks.camera_callback(callback, &cmd_get_settings) != CAMERA_OK) {
        ESP_LOGW(TAG, "Failed to retrieve settings from the callback");
        return ESP_FAIL;
    }

    int video_busy = cmd_get_settings.recording_mode == RECORDING_MODE_VIDEO && cmd_get_settings.processing;
    int image_busy = cmd_get_settings.recording_mode == RECORDING_MODE_STILL && cmd_get_settings.processing;

    mavlink_msg_camera_capture_status_send(
            MAVLINK_COMM_0,
            get_time_since_boot_in_ms(),
            image_busy, // 0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress
            video_busy, // 0: idle, 1: capture in progress
            0,
            cmd_get_settings.recording_time_ms,
            cmd_get_settings.available_capacity,
            cmd_get_settings.image_count
            );

    return ESP_OK;
}

esp_err_t mavlink_camera_send_storage_information(uint8_t storage_id) {
    if (storage_id > 1) {
        // We don't have more than one storage for now
        return ESP_FAIL;
    }

    mavlink_msg_storage_information_send(
            MAVLINK_COMM_0,
            get_time_since_boot_in_ms(),
            1,
            1,
            STORAGE_STATUS_NOT_SUPPORTED,
            0, 0, 0, 0, 0,
            STORAGE_TYPE_UNKNOWN, NULL
            );

    return ESP_OK;
}

esp_err_t mavlink_camera_start_video_recording() {
    mavlink_camera_callback_t callback = {
            .type = CMD_START_VIDEO_RECORDING
    };

    if (mavlink_callbacks.camera_callback(callback, NULL) != CAMERA_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mavlink_camera_stop_video_recording() {
    mavlink_camera_callback_t callback = {
            .type = CMD_STOP_VIDEO_RECORDING
    };

    if (mavlink_callbacks.camera_callback(callback, NULL) != CAMERA_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mavlink_camera_start_capture() {
    mavlink_camera_callback_t callback = {
            .type = CMD_START_STILL_RECORDING
    };

    image_captured_t image_captured;
    bool capture_result = mavlink_callbacks.camera_callback(callback, &image_captured) == CAMERA_OK;

    float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    mavlink_msg_camera_image_captured_send(
            MAVLINK_COMM_0,
            get_time_since_boot_in_ms(),
            0,
            0,
            0, 0, 0, 0, q,
            image_captured.image_index,
            capture_result,
            NULL
            );

    return ESP_OK;
}