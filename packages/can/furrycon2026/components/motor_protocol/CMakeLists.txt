# Component responsibility:
# Registers the motor_protocol scaffold sources and public include directory
# with ESP-IDF so the component is available to the project build.

idf_component_register(
    SRCS
        "motor_codec.c"
        "motor_cmd.c"
        "motor_rx.c"
        "motor_names.c"
    INCLUDE_DIRS
        "."
        "include"
    REQUIRES
        driver
)
