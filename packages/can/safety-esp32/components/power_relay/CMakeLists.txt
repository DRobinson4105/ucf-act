# power_relay: 24V power rail relay/transistor control

idf_component_register(
    SRCS "power_relay.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
