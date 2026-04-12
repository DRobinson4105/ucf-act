# Makefile — Host-native unit test build rules for CAN bus libraries.
#
# NOTE: Include paths in the build rules below must stay in sync with the
# corresponding CMakeLists.txt REQUIRES / INCLUDE_DIRS for each component.
# If a component adds a new dependency in its CMakeLists.txt, add the
# matching -I flag here so host tests see the same headers.

CXX = g++
CXXFLAGS = -Wall -Wextra -Werror -std=c++17 -g -O0
BUILD_DIR = build
INCLUDES = -I../common/protocol/can_protocol/include \
           -I../common/protocol/orin_link_protocol/include \
           -I../control-esp32/components/control_types/include -I../control-esp32/main/logic

MOCK_SRC    = mocks/esp_idf_mock.cpp
SAFETY_SRC  = ../safety-esp32/main/logic/safety_logic.cpp
OBSTACLE_FILTER_SRC = ../safety-esp32/components/ultrasonic_a02yyuw/obstacle_filter.cpp
CONTROL_SRC = ../control-esp32/main/logic/control_logic.cpp
SYSTEM_STATE_SRC = ../safety-esp32/main/logic/system_state_machine.cpp
HEARTBEAT_MON_SRC = ../common/services/heartbeat_monitor/heartbeat_monitor.cpp
ORIN_LINK_PROTOCOL_SRC = ../common/protocol/orin_link_protocol/orin_link_protocol.cpp
MOTOR_PROTO_DIR = ../control-esp32/components/motor_uim2852/src
MOTOR_PROTO_SRC = $(MOTOR_PROTO_DIR)/motor_cmd.cpp \
                  $(MOTOR_PROTO_DIR)/motor_codec.cpp \
                  $(MOTOR_PROTO_DIR)/motor_rx.cpp
ADC_12BITSAR_SRC = ../control-esp32/components/adc_12bitsar/adc_12bitsar.cpp
OPTOCOUPLER_PC817_SRC = ../control-esp32/components/optocoupler_pc817/optocoupler_pc817.cpp
RELAY_SRC = ../control-esp32/components/relay_dpdt_my5nj/relay_dpdt_my5nj.cpp
CONTROL_DRIVER_INPUTS_SRC = ../control-esp32/main/inputs/control_driver_inputs.cpp

# Mock include path must come FIRST so esp_log.h, can_twai.h, etc. resolve to mocks
MOCK_INCLUDES = -Imocks
DRIVER_INPUTS_CXXFLAGS = $(CXXFLAGS) -Wno-unused-variable
RELAY_CXXFLAGS = $(CXXFLAGS) -Wno-unused-variable

all: $(BUILD_DIR)/test_can_protocol $(BUILD_DIR)/test_orin_link_protocol $(BUILD_DIR)/test_motor_uim2852_protocol $(BUILD_DIR)/test_safety_logic $(BUILD_DIR)/test_obstacle_filter $(BUILD_DIR)/test_control_logic $(BUILD_DIR)/test_system_state $(BUILD_DIR)/test_heartbeat_monitor $(BUILD_DIR)/test_driver_inputs $(BUILD_DIR)/test_control_driver_inputs $(BUILD_DIR)/test_relay $(BUILD_DIR)/test_integration_state_machine
	@set -eu; \
	tmp_summary=$$(mktemp); \
	tmp_out=$$(mktemp); \
	trap 'rm -f "$$tmp_summary" "$$tmp_out"' EXIT INT TERM; \
	total_pass=0; \
	total_tests=0; \
	overall_fail=0; \
	run_suite() { \
		label="$$1"; \
		binary="$$2"; \
		echo "--- Running $$label tests ---"; \
		if "$$binary" >"$$tmp_out" 2>&1; then \
			rc=0; \
		else \
			rc=$$?; \
			overall_fail=1; \
		fi; \
		cat "$$tmp_out"; \
		echo ""; \
		counts=$$(awk '/tests[[:space:]]+passed/ { line=$$0; gsub(/[^0-9\/]/, "", line); if (line ~ /^[0-9]+\/[0-9]+$$/) { split(line, parts, "/"); p=parts[1]; t=parts[2]; } } END { if (p != "") printf "%s %s", p, t }' "$$tmp_out"); \
		if [ -n "$$counts" ]; then \
			passed=$${counts% *}; \
			total=$${counts#* }; \
		else \
			passed=0; \
			total=0; \
			overall_fail=1; \
			rc=1; \
		fi; \
		total_pass=$$((total_pass + passed)); \
		total_tests=$$((total_tests + total)); \
		status="PASS"; \
		if [ $$rc -ne 0 ]; then \
			status="FAIL"; \
		fi; \
		printf "%s|%s|%s|%s\n" "$$label" "$$passed" "$$total" "$$status" >> "$$tmp_summary"; \
	}; \
	run_suite "can_protocol" ./$(BUILD_DIR)/test_can_protocol; \
	run_suite "orin_link_protocol" ./$(BUILD_DIR)/test_orin_link_protocol; \
	run_suite "motor_uim2852_protocol" ./$(BUILD_DIR)/test_motor_uim2852_protocol; \
	run_suite "safety_logic" ./$(BUILD_DIR)/test_safety_logic; \
	run_suite "obstacle_filter" ./$(BUILD_DIR)/test_obstacle_filter; \
	run_suite "control_logic" ./$(BUILD_DIR)/test_control_logic; \
	run_suite "system_state" ./$(BUILD_DIR)/test_system_state; \
	run_suite "heartbeat_monitor" ./$(BUILD_DIR)/test_heartbeat_monitor; \
	run_suite "driver_inputs" ./$(BUILD_DIR)/test_driver_inputs; \
	run_suite "control_driver_inputs" ./$(BUILD_DIR)/test_control_driver_inputs; \
	run_suite "relay" ./$(BUILD_DIR)/test_relay; \
	run_suite "integration_state_machine" ./$(BUILD_DIR)/test_integration_state_machine; \
	if [ $$total_pass -ne $$total_tests ]; then \
		overall_fail=1; \
	fi; \
	echo ""; \
	echo "=============================== TEST SUMMARY ==============================="; \
	printf "%-28s %-11s %s\n" "Suite" "Passed" "Status"; \
	echo "-----------------------------------------------------------------------------"; \
	while IFS='|' read -r label passed total status; do \
		ratio="$$passed/$$total"; \
		printf "%-28s %-11s %s\n" "$$label" "$$ratio" "$$status"; \
	done < "$$tmp_summary"; \
	echo "-----------------------------------------------------------------------------"; \
	overall_status="PASS"; \
	if [ $$overall_fail -ne 0 ]; then \
		overall_status="FAIL"; \
	fi; \
	total_ratio="$$total_pass/$$total_tests"; \
	printf "%-28s %-11s %s\n" "TOTAL" "$$total_ratio" "$$overall_status"; \
	echo "============================================================================="; \
	echo ""; \
	if [ $$overall_fail -ne 0 ]; then \
		exit 1; \
	fi

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

test_can_protocol: $(BUILD_DIR)/test_can_protocol

$(BUILD_DIR)/test_can_protocol: test_can_protocol.cpp | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

test_orin_link_protocol: $(BUILD_DIR)/test_orin_link_protocol

$(BUILD_DIR)/test_orin_link_protocol: test_orin_link_protocol.cpp $(ORIN_LINK_PROTOCOL_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

test_motor_uim2852_protocol: $(BUILD_DIR)/test_motor_uim2852_protocol

$(BUILD_DIR)/test_motor_uim2852_protocol: test_motor_uim2852_protocol.cpp $(MOTOR_PROTO_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -Imocks -I../control-esp32/components/motor_uim2852/include -I../control-esp32/components/motor_uim2852/src $(INCLUDES) -include motor_protocol.h -o $@ $^

test_safety_logic: $(BUILD_DIR)/test_safety_logic

$(BUILD_DIR)/test_safety_logic: test_safety_logic.cpp $(SAFETY_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -I../safety-esp32/main/logic $(INCLUDES) -o $@ $^

test_obstacle_filter: $(BUILD_DIR)/test_obstacle_filter

$(BUILD_DIR)/test_obstacle_filter: test_obstacle_filter.cpp $(OBSTACLE_FILTER_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -I../safety-esp32/components/ultrasonic_a02yyuw/include $(INCLUDES) -o $@ $^

test_control_logic: $(BUILD_DIR)/test_control_logic

$(BUILD_DIR)/test_control_logic: test_control_logic.cpp $(CONTROL_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -I../control-esp32/components/control_types/include -I../control-esp32/main/logic $(INCLUDES) -o $@ $^

test_system_state: $(BUILD_DIR)/test_system_state

$(BUILD_DIR)/test_system_state: test_system_state.cpp $(SYSTEM_STATE_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -I../safety-esp32/main/logic $(INCLUDES) -o $@ $^

$(BUILD_DIR)/esp_idf_mock.o: mocks/esp_idf_mock.cpp | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -Imocks -c $< -o $@

test_heartbeat_monitor: $(BUILD_DIR)/test_heartbeat_monitor

$(BUILD_DIR)/test_heartbeat_monitor: test_heartbeat_monitor.cpp $(HEARTBEAT_MON_SRC) $(BUILD_DIR)/esp_idf_mock.o | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -Imocks -I../common/services/heartbeat_monitor/include $(INCLUDES) -o $@ test_heartbeat_monitor.cpp $(HEARTBEAT_MON_SRC) $(BUILD_DIR)/esp_idf_mock.o

test_driver_inputs: $(BUILD_DIR)/test_driver_inputs

$(BUILD_DIR)/test_driver_inputs: test_driver_inputs.cpp $(ADC_12BITSAR_SRC) $(OPTOCOUPLER_PC817_SRC) $(BUILD_DIR)/esp_idf_mock.o | $(BUILD_DIR)
	@$(CXX) $(DRIVER_INPUTS_CXXFLAGS) -Imocks -I../control-esp32/components/adc_12bitsar/include -I../control-esp32/components/optocoupler_pc817/include $(INCLUDES) -o $@ test_driver_inputs.cpp $(ADC_12BITSAR_SRC) $(OPTOCOUPLER_PC817_SRC) $(BUILD_DIR)/esp_idf_mock.o

test_control_driver_inputs: $(BUILD_DIR)/test_control_driver_inputs

$(BUILD_DIR)/test_control_driver_inputs: test_control_driver_inputs.cpp $(CONTROL_DRIVER_INPUTS_SRC) $(ADC_12BITSAR_SRC) $(OPTOCOUPLER_PC817_SRC) $(BUILD_DIR)/esp_idf_mock.o | $(BUILD_DIR)
	@$(CXX) $(DRIVER_INPUTS_CXXFLAGS) -Imocks -I../control-esp32/main -I../control-esp32/main/inputs -I../control-esp32/components/adc_12bitsar/include -I../control-esp32/components/optocoupler_pc817/include $(INCLUDES) -o $@ test_control_driver_inputs.cpp $(CONTROL_DRIVER_INPUTS_SRC) $(ADC_12BITSAR_SRC) $(OPTOCOUPLER_PC817_SRC) $(BUILD_DIR)/esp_idf_mock.o

test_relay: $(BUILD_DIR)/test_relay

$(BUILD_DIR)/test_relay: test_relay.cpp $(RELAY_SRC) $(BUILD_DIR)/esp_idf_mock.o | $(BUILD_DIR)
	@$(CXX) $(RELAY_CXXFLAGS) -Imocks -I../control-esp32/components/relay_dpdt_my5nj/include $(INCLUDES) -o $@ test_relay.cpp $(RELAY_SRC) $(BUILD_DIR)/esp_idf_mock.o

test_integration_state_machine: $(BUILD_DIR)/test_integration_state_machine

$(BUILD_DIR)/test_integration_state_machine: test_integration_state_machine.cpp $(SYSTEM_STATE_SRC) $(CONTROL_SRC) | $(BUILD_DIR)
	@$(CXX) $(CXXFLAGS) -I../safety-esp32/main/logic -I../control-esp32/components/control_types/include -I../control-esp32/main/logic $(INCLUDES) -o $@ $^

clean:
	@rm -rf $(BUILD_DIR)

.PHONY: all clean test_can_protocol test_orin_link_protocol test_motor_uim2852_protocol test_safety_logic test_obstacle_filter test_control_logic test_system_state test_heartbeat_monitor test_driver_inputs test_control_driver_inputs test_relay test_integration_state_machine
