idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol heartbeat throttle_mux stepper_motor_uim2852 enable_relay override_sensors control_logic debug_console
    PRIV_REQUIRES esp_timer
)
