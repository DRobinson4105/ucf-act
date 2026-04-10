# Component responsibility:
# Registers the motor_diag semantic description and rendering layer.

idf_component_register(
    SRCS
        "src/motor_diag_describe_cmd.c"
        "src/motor_diag_describe_rx.c"
        "src/motor_diag_render.c"
        "src/motor_diag_tables.c"
    INCLUDE_DIRS
        "include"
        "src"
    REQUIRES
        log
        motor_protocol
)
