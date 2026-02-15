# optocoupler_pc817: F/R switch decoding and debounce for PC817 channels

idf_component_register(
    SRCS "optocoupler_pc817.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver can_protocol
)
