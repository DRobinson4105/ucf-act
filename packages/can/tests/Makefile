# Makefile — Host-native unit test build rules for CAN bus libraries.

CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -Werror -std=c11 -g -O0
CXXFLAGS = -Wall -Wextra -Werror -std=c++17 -g -O0
INCLUDES = -I../common/protocol/stepper_protocol_uim2852/include \
           -I../common/protocol/can_protocol/include

STEPPER_SRC = ../common/protocol/stepper_protocol_uim2852/stepper_protocol_uim2852.c
MOTOR_SRC   = ../control-esp32/components/stepper_motor_uim2852/stepper_motor_uim2852.c
MOCK_SRC    = mocks/esp_idf_mock.c
SAFETY_SRC  = ../common/logic/safety_logic/safety_logic.c
CONTROL_SRC = ../common/logic/control_logic/control_logic.c
SYSTEM_STATE_SRC = ../common/logic/system_state/system_state.c
HEARTBEAT_MON_SRC = ../common/services/heartbeat_monitor/heartbeat_monitor.cpp
ADC_12BITSAR_SRC = ../control-esp32/components/adc_12bitsar/adc_12bitsar.cpp
OPTOCOUPLER_PC817_SRC = ../control-esp32/components/optocoupler_pc817/optocoupler_pc817.cpp

# Mock include path must come FIRST so esp_log.h, can_twai.hh, etc. resolve to mocks
MOCK_INCLUDES = -Imocks \
                -I../control-esp32/components/stepper_motor_uim2852/include

# Motor component has an unused TAG variable when logging is mocked out
MOTOR_CFLAGS = $(CFLAGS) -Wno-unused-variable
DRIVER_INPUTS_CXXFLAGS = $(CXXFLAGS) -Wno-unused-variable

all: test_stepper_protocol test_can_protocol test_motor_component test_safety_logic test_control_logic test_system_state test_heartbeat_monitor test_driver_inputs
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
	run_suite "stepper_protocol" ./test_stepper_protocol; \
	run_suite "can_protocol" ./test_can_protocol; \
	run_suite "motor_component" ./test_motor_component; \
	run_suite "safety_logic" ./test_safety_logic; \
	run_suite "control_logic" ./test_control_logic; \
	run_suite "system_state" ./test_system_state; \
	run_suite "heartbeat_monitor" ./test_heartbeat_monitor; \
	run_suite "driver_inputs" ./test_driver_inputs; \
	if [ $$total_pass -ne $$total_tests ]; then \
		overall_fail=1; \
	fi; \
	echo ""; \
	echo "============================== TEST SUMMARY =============================="; \
	printf "%-24s %-11s %s\n" "Suite" "Passed" "Status"; \
	echo "-------------------------------------------------------------------------"; \
	while IFS='|' read -r label passed total status; do \
		ratio="$$passed/$$total"; \
		printf "%-24s %-11s %s\n" "$$label" "$$ratio" "$$status"; \
	done < "$$tmp_summary"; \
	echo "-------------------------------------------------------------------------"; \
	overall_status="PASS"; \
	if [ $$overall_fail -ne 0 ]; then \
		overall_status="FAIL"; \
	fi; \
	total_ratio="$$total_pass/$$total_tests"; \
	printf "%-24s %-11s %s\n" "TOTAL" "$$total_ratio" "$$overall_status"; \
	echo "========================================================================="; \
	echo ""; \
	if [ $$overall_fail -ne 0 ]; then \
		exit 1; \
	fi

test_stepper_protocol: test_stepper_protocol.c $(STEPPER_SRC)
	@$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^

test_can_protocol: test_can_protocol.c
	@$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^

test_motor_component: test_motor_component.c $(STEPPER_SRC) $(MOTOR_SRC) $(MOCK_SRC)
	@$(CC) $(MOTOR_CFLAGS) $(MOCK_INCLUDES) $(INCLUDES) -o $@ $^

test_safety_logic: test_safety_logic.c $(SAFETY_SRC)
	@$(CC) $(CFLAGS) -I../common/logic/safety_logic/include $(INCLUDES) -o $@ $^

test_control_logic: test_control_logic.c $(CONTROL_SRC)
	@$(CC) $(CFLAGS) -I../common/logic/control_logic/include $(INCLUDES) -o $@ $^

test_system_state: test_system_state.c $(SYSTEM_STATE_SRC)
	@$(CC) $(CFLAGS) -I../common/logic/system_state/include $(INCLUDES) -o $@ $^

esp_idf_mock.o: mocks/esp_idf_mock.c
	@$(CC) $(CFLAGS) -Imocks -c $< -o $@

test_heartbeat_monitor: test_heartbeat_monitor.cpp $(HEARTBEAT_MON_SRC) esp_idf_mock.o
	@$(CXX) $(CXXFLAGS) -Imocks -I../common/services/heartbeat_monitor/include -o $@ test_heartbeat_monitor.cpp $(HEARTBEAT_MON_SRC) esp_idf_mock.o

test_driver_inputs: test_driver_inputs.cpp $(ADC_12BITSAR_SRC) $(OPTOCOUPLER_PC817_SRC) esp_idf_mock.o
	@$(CXX) $(DRIVER_INPUTS_CXXFLAGS) -Imocks -I../control-esp32/components/adc_12bitsar/include -I../control-esp32/components/optocoupler_pc817/include -I../common/protocol/can_protocol/include -o $@ test_driver_inputs.cpp $(ADC_12BITSAR_SRC) $(OPTOCOUPLER_PC817_SRC) esp_idf_mock.o

clean:
	@rm -f test_stepper_protocol test_can_protocol test_motor_component test_safety_logic test_control_logic test_system_state test_heartbeat_monitor test_driver_inputs esp_idf_mock.o

.PHONY: all clean
