//
// Created by Hugo Trippaers on 04/01/2021.
//

#ifndef GOPROCONTROL_GOPRO_WOL_H
#define GOPROCONTROL_GOPRO_WOL_H

esp_err_t gopro_wol_send(char *mac_address);
esp_err_t gopro_keepalive_send();

#endif //GOPROCONTROL_GOPRO_WOL_H
