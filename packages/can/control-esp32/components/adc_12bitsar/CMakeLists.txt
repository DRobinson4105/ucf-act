# adc_12bitsar: calibrated ESP32-C6 12-bit SAR ADC helper

idf_component_register(
    SRCS "adc_12bitsar.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc
)
