idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol led_ws2812 heartbeat_monitor gpio_input relay_srd05vdc ultrasonic_a02yyuw safety_logic system_state
    PRIV_REQUIRES esp_timer
)
