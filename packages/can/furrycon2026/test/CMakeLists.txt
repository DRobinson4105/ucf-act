# Keep the Unity tests always compiled, and add the test-runner app entrypoint
# only when the dedicated test firmware path is enabled.
set(test_srcs
    "test_motor_codec.c"
    "test_motor_cmd.c"
    "test_motor_diag.c"
    "test_motor_dispatch.c"
    "test_motor_exec.c"
    "test_motor_setup.c"
    "test_motor_rx.c"
    "test_twai_port.c"
)

if(MOTOR_PROTOCOL_TEST_RUNNER)
    list(APPEND test_srcs "test_runner_main.c")
endif()

# Component responsibility:
# Registers Unity-based project tests for local protocol mechanics.

idf_component_register(
    SRCS
        ${test_srcs}
    INCLUDE_DIRS
        "."
        "../components/motor_dispatch"
        "../components/motor_dispatch/include"
        "../components/motor_diag/include"
        "../components/motor_exec/include"
        "../components/motor_exec/src"
        "../components/motor_setup/include"
        "../components/motor_setup/src"
        "../components/motor_protocol"
        "../components/motor_protocol/include"
    REQUIRES
        motor_dispatch
        motor_diag
        motor_exec
        motor_setup
        unity
        motor_protocol
        twai_port
    WHOLE_ARCHIVE
)
