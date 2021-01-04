set(gopro_srcs "src/gopro.c" "src/gopro_wifi.c" "src/gopro_http.c" "src/gopro_wol.c")

idf_component_register(SRCS "${gopro_srcs}"
        INCLUDE_DIRS "include"
        PRIV_INCLUDE_DIRS "priv_include"
        REQUIRES esp_http_client json)