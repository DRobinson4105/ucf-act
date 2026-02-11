idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol heartbeat multiplexer_dg408djz stepper_motor_uim2852 relay_jd2912 override_sensors control_logic
    PRIV_REQUIRES esp_timer
)
