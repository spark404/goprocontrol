//
// Created by Hugo Trippaers on 04/01/2021.
//

#include "esp_system.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"

static const char *TAG = "gopro_wol";

esp_err_t gopro_wol_send(char *mac_address) {
    ESP_LOGI(TAG, "trying to send WOL for mac %s", mac_address);
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(9);

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    //f6dd9e435208
    uint8_t payload[102];
    for (int i = 0; i < 6; i++) {
        payload[i] = 0xff;
    }
    for (int i = 0; i < 16; i++) {
        int index = 6 + 6 * i;
        for (int j = 0; j < 6; j++) {
            int c1 = *(mac_address + j * 2) - 48;
            int c2 = *(mac_address + j * 2 + 1) - 48;
            uint8_t byte = c1 > 9 ? (c1 - 39) << 4 : c1 << 4;
            byte |= c2 > 9 ? c2 - 39 : c2;
            payload[index+j] = byte;
        }
    }
    ESP_LOG_BUFFER_HEX(TAG, payload, 102);

    int err = sendto(sock, payload, 102, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    close(sock);

    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WOL sent for mac %s", mac_address);
    return ESP_OK;
}

esp_err_t gopro_keepalive_send() {
    ESP_LOGI(TAG, "trying to send keep-alive");
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = inet_addr("10.5.5.9");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8554);

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    char *keepalive = "_GPHD_:0:0:2:0.000000\n";
    ESP_LOG_BUFFER_HEX(TAG, keepalive, 22);

    int err = sendto(sock, keepalive, 22, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    close(sock);

    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Keep alive sent");
    return ESP_OK;
}

