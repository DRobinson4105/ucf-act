idf_component_register(
    SRCS "control_logic.cpp"
    INCLUDE_DIRS "include"
    REQUIRES can_protocol control_types
)
