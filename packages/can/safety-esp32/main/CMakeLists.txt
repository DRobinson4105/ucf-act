idf_component_register(
    SRCS
        "main.cpp"
        "state/safety_globals.cpp"
        "health/safety_health.cpp"
        "health/safety_recovery.cpp"
        "tasks/safety_loop.cpp"
        "init/safety_init.cpp"
        "init/safety_startup_log.cpp"
        "transport/safety_can_rx.cpp"
        "transport/safety_heartbeat_tx.cpp"
        "transport/safety_orin_link.cpp"
        "inputs/safety_local_inputs.cpp"
        "logic/safety_logic.cpp"
        "logic/system_state_machine.cpp"
    INCLUDE_DIRS "." "config" "state" "health" "tasks" "init" "transport" "inputs" "logic"
    REQUIRES can_twai can_protocol orin_link_protocol status_led heartbeat_monitor relay_srd05vdc ultrasonic_a02yyuw node_support can_runtime battery_monitor usb_serial_link
    PRIV_REQUIRES esp_timer esp_adc
)
