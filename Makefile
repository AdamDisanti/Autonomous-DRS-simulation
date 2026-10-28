# Active aero supervisor: build the portable C core and run the Python simulation.
# SIMULATION ONLY: nothing here talks to real hardware or a real CAN bus.

PY      ?= .venv/bin/python
BUILD   := build
# macOS: point CMake at the active Xcode SDK (avoids a linker/SDK mismatch).
SYSROOT := $(shell [ "$$(uname)" = Darwin ] && echo -DCMAKE_OSX_SYSROOT=$$(xcrun --sdk macosx --show-sdk-path))

.PHONY: all setup build test test-c test-py scenarios dashboard clean

all: build test

setup:
	python3 -m venv .venv
	$(PY) -m pip install -q pytest matplotlib

build:
	cmake -S core -B $(BUILD) -DCMAKE_BUILD_TYPE=Debug $(SYSROOT) > /dev/null
	cmake --build $(BUILD)

test: test-c test-py

test-c: build
	./$(BUILD)/test_core_c

test-py: build
	$(PY) -m pytest -q tests

scenarios: build
	$(PY) scripts/run_scenario.py

dashboard: build
	$(PY) sim/dashboard/server.py

clean:
	rm -rf $(BUILD) results .pytest_cache
