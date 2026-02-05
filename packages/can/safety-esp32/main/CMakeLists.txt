idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol heartbeat heartbeat_monitor nc_mushroom power_relay ultrasonic wireless_remote
)
