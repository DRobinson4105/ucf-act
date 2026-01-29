idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES nc_mushroom wireless_remote safety_can ultrasonic heartbeat
)
