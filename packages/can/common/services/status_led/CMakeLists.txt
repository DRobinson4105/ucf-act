idf_component_register(
    SRCS "status_led.cpp"
    INCLUDE_DIRS "include"
    REQUIRES can_protocol led_ws2812
)
