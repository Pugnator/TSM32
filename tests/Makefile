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

.PHONY: all test clean

all: test

test: $(BLINKER_TEST) $(ENGINE_TEST) $(AHRS_TEST)
	$(BLINKER_TEST)
	$(ENGINE_TEST)
	$(AHRS_TEST)

$(BLINKER_TEST): test_blinker.cc $(ROOT)/Core/Src/switch_ctrl.cc $(ROOT)/Core/Src/turn_ctrl.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(ENGINE_TEST): test_engine_state.cc $(ROOT)/Core/Src/engine_state.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(AHRS_TEST): test_ahrs_math.cc | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(BIN_DIR):
	mkdir -p $@

clean:
	rm -rf $(BIN_DIR)
