# throttle_mux: DG408 analog multiplexer + relay control for throttle

idf_component_register(
    SRCS "throttle_mux.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
