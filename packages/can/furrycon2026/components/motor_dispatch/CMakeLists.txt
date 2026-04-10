idf_component_register(SRCS "motor_dispatch.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES motor_protocol twai_port log freertos)
