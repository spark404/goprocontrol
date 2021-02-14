#include <esp_err.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

/* mavlink_system needs to be set before using any of the mavlink
 * functions.
 */
#include "mavlink_types.h"
mavlink_system_t mavlink_system = {
    CONFIG_MAVLINK_SYSTEM_ID, // System ID (1-255)
    CONFIG_MAVLINK_COMPONENT_ID  // Component ID (a MAV_COMPONENT value)
};

#include "mavlink_bridge.h"
#include "mavlink_handlers.h"
#include "mavlink_camera.h"

static const char *TAG = "mavlink_v2";
static TaskHandle_t heartbeat_task;

#define MAV_LOG_CMDD(tag, name, cmd) ESP_LOGD(tag, "Received %s (p1 %f, p2 %f, p3 %f, p4 %f, p5 %f, p6 %f, p7 %f", \
    name, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5,cmd.param6, cmd.param7)

static void mavlink_message_handler_task(void *pvParameters);
static void mavlink_heartbeat_task(void *pvParameters);

/* This will wrap at some point in the future
 * Celebrate when that happens
 */
uint32_t get_time_since_boot_in_ms()
{
    int64_t current = esp_timer_get_time();
    return current / 1000;  // ESP Timer is in microseconds
}

esp_err_t mavlink_handler_configure(mavlink_config_t mavlink_config)
{
    mavlink_camera_init(&(mavlink_config->camera_callbacks));

    TaskHandle_t task;
    xTaskCreate(mavlink_message_handler_task, "mavlink_message_handler_task", 8192, mavlink_config, 11, &task);

    if (task != NULL) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

_Noreturn static void mavlink_heartbeat_task(void *pvParameters)
{
    assert(pvParameters != NULL);

    mavlink_config_t mavlink_config = (mavlink_config_t)pvParameters;
    mavlink_component_status_t previous_status = MAVLINK_COMPONENT_STATUS_UNKNOWN;

    for (;;) {
        mavlink_component_status_t current_status = mavlink_config->status_callback();
        if (current_status != previous_status) {
            ESP_LOGD(TAG, "Component state changed from %d to %d", previous_status, current_status);
            previous_status = current_status;
        }

        if (current_status == MAVLINK_COMPONENT_STATUS_READY) {
            mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_CAMERA, MAV_AUTOPILOT_INVALID,
                                       MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0,
                                       MAV_STATE_ACTIVE);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

_Noreturn static void mavlink_message_handler_task(void *pvParameters)
{
    assert(pvParameters != NULL);

    mavlink_config_t mavlink_config = (mavlink_config_t)pvParameters;

    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    for (;;) {
        if (xQueueReceive(mavlink_config->incoming_queue, (void *) &msg, (portTickType) portMAX_DELAY)) {
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                if (msg.compid == 1 && heartbeat_task == NULL) {
                    ESP_LOGI(TAG, "Received new heartbeat from FlightController with sysid %d", msg.sysid);
                    if (mavlink_system.sysid != msg.sysid) {
                        ESP_LOGI(TAG, "Changing our sysid from %d to %d", mavlink_system.sysid, msg.sysid);
                        mavlink_system.sysid = msg.sysid;
                    }
                    xTaskCreate(mavlink_heartbeat_task, "mavlink_heartbeat_task", 4096, mavlink_config, 11, &heartbeat_task);
                    ESP_LOGI(TAG, "Started heartbeat task");
                }
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                mavlink_msg_command_long_decode(&msg, &cmd);
                ESP_LOGD(TAG, "Command received: (sysid: %d compid: %d msgid: %d) from compid %d, sysid %d",
                         cmd.target_system, cmd.target_component, cmd.command, msg.compid, msg.sysid);

                if (cmd.target_system != mavlink_system.sysid || cmd.target_component != mavlink_system.compid) {
                    // Not for us
                    break;
                }

                switch (cmd.command) {
                case MAV_CMD_REQUEST_MESSAGE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_MESSAGE", cmd);
                    uint32_t requested_message_id = cmd.param1;
                    switch (requested_message_id) {
                    case MAVLINK_MSG_ID_CAMERA_INFORMATION:
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                        mavlink_camera_send_camera_information();
                        break;
                    case MAVLINK_MSG_ID_CAMERA_SETTINGS:
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                        mavlink_camera_send_camera_settings();
                        break;
                    case MAVLINK_MSG_ID_STORAGE_INFORMATION:
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                        mavlink_camera_send_storage_information(cmd.param2);
                        break;
                    case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                        mavlink_camera_send_camera_capture_status();
                        break;
                    default:
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_UNSUPPORTED, 255, 0, msg.sysid, msg.compid);
                        break;
                    }
                    break;
                case MAV_CMD_REQUEST_CAMERA_INFORMATION:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_CAMERA_INFORMATION", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                    mavlink_camera_send_camera_information();
                    break;
                case MAV_CMD_REQUEST_CAMERA_SETTINGS:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_CAMERA_SETTINGS", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                    mavlink_camera_send_camera_settings();
                    break;
                case MAV_CMD_REQUEST_STORAGE_INFORMATION:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_STORAGE_INFORMATION", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                    mavlink_camera_send_storage_information(cmd.param2);
                    break;
                case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 255, 0, msg.sysid, msg.compid);
                    mavlink_camera_send_camera_capture_status();
                    break;
                case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_UNSUPPORTED, 255, 0, msg.sysid, msg.compid);
                    break;
                case MAV_CMD_SET_CAMERA_MODE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_SET_CAMERA_MODE", cmd);
                    if (mavlink_camera_handle_set_mode(cmd.param2) != ESP_OK) {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_FAILED, 255, 0, msg.sysid, msg.compid);
                    } else {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    }
                    break;
                case MAV_CMD_IMAGE_START_CAPTURE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_IMAGE_START_CAPTURE", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    mavlink_camera_handle_start_capture();
                    break;
                case MAV_CMD_IMAGE_STOP_CAPTURE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_IMAGE_STOP_CAPTURE", cmd);
                    // No support for intervals yet
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    mavlink_camera_handle_stop_capture();
                    break;
                case MAV_CMD_VIDEO_START_CAPTURE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_VIDEO_START_CAPTURE", cmd);
                    if (mavlink_camera_handle_start_video_recording() == ESP_OK) {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    } else {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_FAILED, 0, 0,
                                                     msg.sysid, msg.compid);
                    }
                    break;
                case MAV_CMD_VIDEO_STOP_CAPTURE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_VIDEO_STOP_CAPTURE", cmd);
                    if (mavlink_camera_handle_stop_video_recording() == ESP_OK) {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    } else {
                        mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_FAILED, 0, 0,
                                                     msg.sysid, msg.compid);
                    }
                    break;
                case MAV_CMD_DO_DIGICAM_CONTROL:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_DO_DIGICAM_CONTROL", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    mavlink_camera_handle_digicam_control(cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6, cmd.param7);
                    break;
                case MAV_CMD_DO_DIGICAM_CONFIGURE:
                    MAV_LOG_CMDD(TAG, "MAV_CMD_DO_DIGICAM_CONFIGURE", cmd);
                    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
                    mavlink_camera_handle_digicam_configure(cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6, cmd.param7);
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
