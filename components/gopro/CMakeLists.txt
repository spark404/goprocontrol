set(gopro_srcs "src/gopro.c")

idf_component_register(SRCS "${gopro_srcs}"
        INCLUDE_DIRS "include"
        REQUIRES esp_http_client)