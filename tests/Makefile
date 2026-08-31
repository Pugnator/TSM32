CXX ?= g++
CXXFLAGS ?= -std=c++17 -O0 -g -Wall -Wextra -pedantic -DDEBUG -D_GNU_SOURCE

ROOT := ..
BIN_DIR := $(ROOT)/build/host-tests
INCLUDES := -I. -I$(ROOT)/Core/Inc -I$(ROOT)/Core/Src/ahrs/inc

ifeq ($(OS),Windows_NT)
EXEEXT := .exe
endif

BLINKER_TEST := $(BIN_DIR)/test_blinker$(EXEEXT)
ENGINE_TEST := $(BIN_DIR)/test_engine_state$(EXEEXT)
AHRS_TEST := $(BIN_DIR)/test_ahrs_math$(EXEEXT)
VOLTAGE_TEST := $(BIN_DIR)/test_voltage_policy$(EXEEXT)
J1850_PARSER_TEST := $(BIN_DIR)/test_j1850_parser$(EXEEXT)
EEPROM_TEST := $(BIN_DIR)/test_eeprom$(EXEEXT)
SECURITY_TEST := $(BIN_DIR)/test_security$(EXEEXT)

.PHONY: all test clean

all: test

test: $(BLINKER_TEST) $(ENGINE_TEST) $(AHRS_TEST) $(VOLTAGE_TEST) $(J1850_PARSER_TEST) $(EEPROM_TEST) $(SECURITY_TEST)
	$(BLINKER_TEST)
	$(ENGINE_TEST)
	$(AHRS_TEST)
	$(VOLTAGE_TEST)
	$(J1850_PARSER_TEST)
	$(EEPROM_TEST)
	$(SECURITY_TEST)

$(BLINKER_TEST): test_blinker.cc $(ROOT)/Core/Src/switch_ctrl.cc $(ROOT)/Core/Src/turn_ctrl.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(ENGINE_TEST): test_engine_state.cc $(ROOT)/Core/Src/engine_state.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(AHRS_TEST): test_ahrs_math.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(VOLTAGE_TEST): test_voltage_policy.cc $(ROOT)/Core/Src/voltage_policy.cc $(ROOT)/Core/Src/bulb_ctrl.cc $(ROOT)/Core/Src/engine_state.cc $(ROOT)/Core/Src/turn_ctrl.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(J1850_PARSER_TEST): test_j1850_parser.cc $(ROOT)/Core/Src/j1850parser.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(EEPROM_TEST): test_eeprom.cc $(ROOT)/Core/Src/eeprom.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(SECURITY_TEST): test_security.cc $(ROOT)/Core/Src/security.cc $(ROOT)/Core/Src/eeprom.cc $(ROOT)/Core/Src/turn_ctrl.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(BIN_DIR):
	mkdir -p $@

clean:
	rm -rf $(BIN_DIR)
