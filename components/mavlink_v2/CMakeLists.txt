set(mavlink_v2_srcs "src/mavlink_v2.c" "src/mavlink_uart.c" "src/mavlink_camera.c")

idf_component_register(SRCS "${mavlink_v2_srcs}"
                       INCLUDE_DIRS "include" "c_library_v2"
                       PRIV_INCLUDE_DIRS "priv_include")