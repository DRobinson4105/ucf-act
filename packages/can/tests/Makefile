# Makefile — Host-native unit test build rules for CAN bus libraries.

CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -Werror -std=c11 -g -O0
CXXFLAGS = -Wall -Wextra -Werror -std=c++17 -g -O0
INCLUDES = -I../common/stepper_protocol_uim2852/include \
           -I../common/can_protocol/include

STEPPER_SRC = ../common/stepper_protocol_uim2852/stepper_protocol_uim2852.c
MOTOR_SRC   = ../control-esp32/components/stepper_motor_uim2852/stepper_motor_uim2852.c
MOCK_SRC    = mocks/esp_idf_mock.c
SAFETY_SRC  = ../common/safety_logic/safety_logic.c
CONTROL_SRC = ../common/control_logic/control_logic.c
SYSTEM_STATE_SRC = ../common/system_state/system_state.c
HEARTBEAT_MON_SRC = ../common/heartbeat_monitor/heartbeat_monitor.cpp
OVERRIDE_SENSORS_SRC = ../control-esp32/components/override_sensors/override_sensors.cpp

# Mock include path must come FIRST so esp_log.h, can_twai.hh, etc. resolve to mocks
MOCK_INCLUDES = -Imocks \
                -I../control-esp32/components/stepper_motor_uim2852/include

# Motor component has an unused TAG variable when logging is mocked out
MOTOR_CFLAGS = $(CFLAGS) -Wno-unused-variable
OVERRIDE_CXXFLAGS = $(CXXFLAGS) -Wno-unused-variable

all: test_stepper_protocol test_can_protocol test_motor_component test_safety_logic test_control_logic test_system_state test_heartbeat_monitor test_override_sensors
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
	run_suite "override_sensors" ./test_override_sensors; \
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
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^

test_can_protocol: test_can_protocol.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^

test_motor_component: test_motor_component.c $(STEPPER_SRC) $(MOTOR_SRC) $(MOCK_SRC)
	$(CC) $(MOTOR_CFLAGS) $(MOCK_INCLUDES) $(INCLUDES) -o $@ $^

test_safety_logic: test_safety_logic.c $(SAFETY_SRC)
	$(CC) $(CFLAGS) -I../common/safety_logic/include $(INCLUDES) -o $@ $^

test_control_logic: test_control_logic.c $(CONTROL_SRC)
	$(CC) $(CFLAGS) -I../common/control_logic/include $(INCLUDES) -o $@ $^

test_system_state: test_system_state.c $(SYSTEM_STATE_SRC)
	$(CC) $(CFLAGS) -I../common/system_state/include $(INCLUDES) -o $@ $^

esp_idf_mock.o: mocks/esp_idf_mock.c
	$(CC) $(CFLAGS) -Imocks -c $< -o $@

test_heartbeat_monitor: test_heartbeat_monitor.cpp $(HEARTBEAT_MON_SRC) esp_idf_mock.o
	$(CXX) $(CXXFLAGS) -Imocks -I../common/heartbeat_monitor/include -o $@ test_heartbeat_monitor.cpp $(HEARTBEAT_MON_SRC) esp_idf_mock.o

test_override_sensors: test_override_sensors.cpp $(OVERRIDE_SENSORS_SRC) esp_idf_mock.o
	$(CXX) $(OVERRIDE_CXXFLAGS) -Imocks -I../control-esp32/components/override_sensors/include -I../common/can_protocol/include -o $@ test_override_sensors.cpp $(OVERRIDE_SENSORS_SRC) esp_idf_mock.o

clean:
	rm -f test_stepper_protocol test_can_protocol test_motor_component test_safety_logic test_control_logic test_system_state test_heartbeat_monitor test_override_sensors esp_idf_mock.o

.PHONY: all clean
