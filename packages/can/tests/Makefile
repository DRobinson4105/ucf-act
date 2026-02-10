# Makefile — Host-native unit test build rules for CAN bus libraries.

CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -g -O0
INCLUDES = -I../common/stepper_protocol_uim2852/include \
           -I../common/can_protocol/include

STEPPER_SRC = ../common/stepper_protocol_uim2852/stepper_protocol_uim2852.c
MOTOR_SRC   = ../control-esp32/components/stepper_motor_uim2852/stepper_motor_uim2852.c
MOCK_SRC    = mocks/esp_idf_mock.c
SAFETY_SRC  = ../common/safety_logic/safety_logic.c
CONTROL_SRC = ../common/control_logic/control_logic.c

# Mock include path must come FIRST so esp_log.h, can_twai.hh, etc. resolve to mocks
MOCK_INCLUDES = -Imocks \
                -I../control-esp32/components/stepper_motor_uim2852/include

# Motor component has an unused TAG variable when logging is mocked out
MOTOR_CFLAGS = $(CFLAGS) -Wno-unused-variable

all: test_stepper_protocol test_can_protocol test_motor_component test_safety_logic test_control_logic
	@echo "--- Running stepper protocol tests ---"
	./test_stepper_protocol
	@echo ""
	@echo "--- Running CAN protocol tests ---"
	./test_can_protocol
	@echo ""
	@echo "--- Running motor component tests ---"
	./test_motor_component
	@echo ""
	@echo "--- Running safety logic tests ---"
	./test_safety_logic
	@echo ""
	@echo "--- Running control logic tests ---"
	./test_control_logic
	@echo ""
	@echo "All tests passed."

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

clean:
	rm -f test_stepper_protocol test_can_protocol test_motor_component test_safety_logic test_control_logic

.PHONY: all clean
