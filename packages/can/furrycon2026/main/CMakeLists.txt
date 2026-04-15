set(main_srcs "app_twai_config.c")

if(NOT MOTOR_PROTOCOL_TEST_RUNNER)
    list(APPEND main_srcs "brakingMotor.c")
endif()

idf_component_register(SRCS ${main_srcs}
                    INCLUDE_DIRS "."
                    REQUIRES twai_port motor_protocol motor_diag motor_dispatch motor_setup motor_exec esp_timer serial_input dac_mcp4728 relay_dpdt_my5nj pwm_motor)
