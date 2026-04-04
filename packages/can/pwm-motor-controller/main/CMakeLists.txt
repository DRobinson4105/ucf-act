idf_component_register(
    SRCS "main.c" "pwm_motor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc log
)
