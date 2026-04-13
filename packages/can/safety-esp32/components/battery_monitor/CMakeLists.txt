idf_component_register(
    SRCS "battery_monitor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc can_protocol nvs_flash
)
