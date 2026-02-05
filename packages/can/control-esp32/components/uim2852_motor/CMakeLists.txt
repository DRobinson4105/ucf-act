# uim2852_motor: UIM2852CA motor control API

idf_component_register(
    SRCS "uim2852_motor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver uim2852_protocol
    PRIV_REQUIRES can_twai
)
