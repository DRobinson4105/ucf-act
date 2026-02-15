# relay_jd2912: JD-2912 pedal bypass relay control (via IRLZ44N MOSFET)

idf_component_register(
    SRCS "relay_jd2912.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
