idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai can_protocol led_ws2812 multiplexer_dg408djz stepper_motor_uim2852 relay_srd05vdc relay_jd2912 adc_12bitsar optocoupler_pc817 control_logic heartbeat_monitor
    PRIV_REQUIRES esp_timer
)
