idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES stepper_protocol_uim2852
    PRIV_REQUIRES driver
)
