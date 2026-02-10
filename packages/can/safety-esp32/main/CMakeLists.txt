idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol heartbeat heartbeat_monitor push_button_hb2es544 power_relay ultrasonic_a02yyuw rf_remote_ev1527 safety_logic system_state
)
