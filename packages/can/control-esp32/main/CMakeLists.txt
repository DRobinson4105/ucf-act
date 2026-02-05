idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol heartbeat throttle_mux uim2852_motor enable_relay override_sensors
    PRIV_REQUIRES esp_timer
)
