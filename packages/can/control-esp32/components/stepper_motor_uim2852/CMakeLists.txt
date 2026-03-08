idf_component_register(
    SRCS "stepper_motor_uim2852.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver stepper_protocol_uim2852
    PRIV_REQUIRES can_twai
)
