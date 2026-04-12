idf_component_register(
    SRCS
        "main.cpp"
        "state/control_globals.cpp"
        "health/control_health.cpp"
        "health/control_recovery.cpp"
        "actuators/control_actuator_runtime.cpp"
        "tasks/control_loop.cpp"
        "init/control_init.cpp"
        "init/control_startup_log.cpp"
        "transport/control_can_rx.cpp"
        "transport/control_heartbeat_tx.cpp"
        "transport/control_orin_link.cpp"
        "inputs/control_driver_inputs.cpp"
        "bench/control_test_mode.cpp"
        "logic/control_logic.cpp"
    INCLUDE_DIRS "." "config" "state" "health" "actuators" "tasks" "init" "transport" "inputs" "bench" "logic"
    REQUIRES can_twai can_protocol orin_link_protocol status_led dac_mcp4728 motor_uim2852 relay_dpdt_my5nj adc_12bitsar optocoupler_pc817 heartbeat_monitor node_support can_runtime serial_input usb_serial_link
    PRIV_REQUIRES esp_timer
)
