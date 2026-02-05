# override_sensors: pedal ADC and F/R optocoupler override detection

idf_component_register(
    SRCS "override_sensors.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc can_protocol
)
