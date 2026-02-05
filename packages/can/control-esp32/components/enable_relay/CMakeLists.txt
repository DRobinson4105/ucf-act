# enable_relay: Pedal bypass relay control for autonomous operation

idf_component_register(
    SRCS "enable_relay.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
