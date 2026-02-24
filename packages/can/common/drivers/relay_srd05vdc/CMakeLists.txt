# relay_srd05vdc: AEDIKO SRD-05VDC-SL-C 5V relay module (GPIO-driven, shared)

idf_component_register(
    SRCS "relay_srd05vdc.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
