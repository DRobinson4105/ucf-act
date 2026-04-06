idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol led_ws2812 dac_mcp4728 stepper_motor_uim2852 relay_dpdt_my5nj adc_12bitsar optocoupler_pc817 control_logic heartbeat_monitor node_utils serial_input
    PRIV_REQUIRES esp_timer
)
