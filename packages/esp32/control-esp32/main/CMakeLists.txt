idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES can_twai heartbeat mcp4728 uim342
)
