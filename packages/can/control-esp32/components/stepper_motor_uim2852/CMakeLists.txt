# stepper_motor_uim2852: UIM2852CA stepper motor control API

idf_component_register(
    SRCS "stepper_motor_uim2852.c"
    INCLUDE_DIRS "include"
    REQUIRES driver stepper_protocol_uim2852
    PRIV_REQUIRES can_twai
)
