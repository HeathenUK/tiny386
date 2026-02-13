# tiny386 ESP32 Build System
# Modeled after trackmatsu build system

PORT ?= /dev/ttyACM0

# Prefer local ESP-IDF if it exists, otherwise use .IDF_PATH file, then env, then default
LOCAL_IDF_PATH := $(shell pwd)/esp-idf
IDF_PATH := $(shell if [ -d "$(LOCAL_IDF_PATH)" ]; then echo "$(LOCAL_IDF_PATH)"; elif [ -f .IDF_PATH ]; then cat .IDF_PATH; elif [ -n "$$IDF_PATH" ]; then echo "$$IDF_PATH"; else echo "$(LOCAL_IDF_PATH)"; fi)
IDF_TOOLS_PATH ?= $(shell cat .IDF_TOOLS_PATH 2>/dev/null || echo `pwd`/esp-idf-tools)
IDF_BRANCH ?= master
IDF_COMMIT ?= aaebc374676621980878789c49d239232ea714c5
IDF_EXPORT_QUIET ?= 1
IDF_GITHUB_ASSETS ?= dl.espressif.com/github_assets
MAKEFLAGS += --silent

SHELL := /usr/bin/env bash

DEVICE ?= tanmatsu
BUILD ?= build/$(DEVICE)

export IDF_TOOLS_PATH
export IDF_GITHUB_ASSETS

# General targets

.PHONY: all
all: build

# Preparation

.PHONY: prepare
prepare: submodules sdk

.PHONY: submodules
submodules:
	if [ ! -f .submodules_update_done ]; then \
		echo "Updating submodules"; \
		git submodule foreach --recursive 'git clean -fd' || true; \
		git submodule update --init --recursive --force; \
		touch .submodules_update_done; \
	fi

.PHONY: sdk
sdk:
	@if test -d "$(IDF_PATH)"; then \
		echo "ESP-IDF directory exists, attempting to repair/complete setup..."; \
		if [ -d "$(IDF_PATH)/.git" ]; then \
			echo "Found existing git repository, updating..."; \
			cd "$(IDF_PATH)"; \
			git fetch origin "$(IDF_COMMIT)" --recurse-submodules || true; \
			git checkout "$(IDF_COMMIT)" || (git reset --hard "$(IDF_COMMIT)" && git clean -fd); \
			git submodule foreach --recursive 'git clean -fd; git reset --hard HEAD' 2>/dev/null || true; \
			git submodule sync --recursive || true; \
			git submodule update --init --recursive --force; \
		else \
			echo "Directory exists but is not a git repository. Removing and starting fresh..."; \
			rm -rf "$(IDF_PATH)"; \
			git clone --recursive --branch "$(IDF_BRANCH)" https://github.com/espressif/esp-idf.git "$(IDF_PATH)" --depth=1 --shallow-submodules; \
			cd "$(IDF_PATH)"; git fetch origin "$(IDF_COMMIT)" --recurse-submodules || true; \
			git checkout "$(IDF_COMMIT)" || (git reset --hard "$(IDF_COMMIT)" && git clean -fd); \
			git submodule foreach --recursive 'git clean -fd; git reset --hard HEAD' 2>/dev/null || true; \
			git submodule sync --recursive || true; \
			git submodule update --init --recursive --force; \
		fi; \
	else \
		echo "Cloning ESP-IDF..."; \
		git clone --recursive --branch "$(IDF_BRANCH)" https://github.com/espressif/esp-idf.git "$(IDF_PATH)" --depth=1 --shallow-submodules; \
		cd "$(IDF_PATH)"; git fetch origin "$(IDF_COMMIT)" --recurse-submodules || true; \
		git checkout "$(IDF_COMMIT)" || (git reset --hard "$(IDF_COMMIT)" && git clean -fd); \
		git submodule foreach --recursive 'git clean -fd; git reset --hard HEAD' 2>/dev/null || true; \
		git submodule sync --recursive || true; \
		git submodule update --init --recursive --force; \
	fi
	@if test -d "$(IDF_TOOLS_PATH)/python_env" && find "$(IDF_TOOLS_PATH)/python_env" -name "bin/python" -type f 2>/dev/null | head -1 | grep -q .; then \
		echo "ESP-IDF tools already installed, skipping..."; \
	else \
		echo "Installing ESP-IDF tools..."; \
		cd "$(IDF_PATH)"; bash install.sh all; \
	fi

# Note: fmopl.inc is no longer needed - ymfm replaced fmopl for OPL emulation

.PHONY: patch_idf
patch_idf:
	@if [ ! -d "$(IDF_PATH)" ]; then \
		echo "ESP-IDF not present at $(IDF_PATH)"; \
		exit 1; \
	fi
	@FFCONF="$(IDF_PATH)/components/fatfs/src/ffconf.h"; \
	if [ ! -f "$$FFCONF" ]; then \
		echo "Cannot find $$FFCONF"; \
		exit 1; \
	fi; \
	if grep -q "^#define[[:space:]]\\+FF_FS_EXFAT[[:space:]]\\+1" "$$FFCONF" && \
	   grep -q "^#define[[:space:]]\\+FF_LBA64[[:space:]]\\+1" "$$FFCONF"; then \
		echo "ESP-IDF exFAT/GPT support already enabled"; \
	else \
		echo "Enabling exFAT + GPT LBA64 in $$FFCONF"; \
		sed -i -E 's/^#define[[:space:]]+FF_FS_EXFAT[[:space:]]+0/#define FF_FS_EXFAT\t\t1/' "$$FFCONF"; \
		sed -i -E 's/^#define[[:space:]]+FF_LBA64[[:space:]]+0/#define FF_LBA64\t\t1/' "$$FFCONF"; \
	fi

.PHONY: patch_bsp
patch_bsp:
	@if [ -d "managed_components/badgeteam__badge-bsp" ]; then \
		if ! grep -q "CONFIG_IDF_TARGET_ESP32" managed_components/badgeteam__badge-bsp/stub/badge_bsp_device.c 2>/dev/null; then \
			echo "Patching badge-bsp for ESP32-P4 compatibility..."; \
			cd managed_components/badgeteam__badge-bsp && patch -p1 < ../../badge-bsp.patch; \
		fi \
	fi

.PHONY: removesdk
removesdk:
	rm -rf "$(IDF_PATH)"
	rm -rf "$(IDF_TOOLS_PATH)"

.PHONY: refreshsdk
refreshsdk: removesdk sdk

.PHONY: menuconfig
menuconfig:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) menuconfig -DDEVICE=$(DEVICE)

# Cleaning

.PHONY: clean
clean:
	rm -rf $(BUILD)
	rm -f .submodules_update_done

.PHONY: fullclean
fullclean: clean
	rm -rf build
	rm -f sdkconfig
	rm -f sdkconfig.old
	rm -f sdkconfig.ci
	rm -f sdkconfig.defaults

# Check if build environment is set up correctly
.PHONY: checkbuildenv
checkbuildenv:
	@if [ -z "$(IDF_PATH)" ]; then echo "IDF_PATH is not set!"; exit 1; fi
	@if [ -z "$(IDF_TOOLS_PATH)" ]; then echo "IDF_TOOLS_PATH is not set!"; exit 1; fi

# Building

.PHONY: build
build: checkbuildenv submodules patch_idf patch_bsp
	export IDF_TOOLS_PATH="$(IDF_TOOLS_PATH)" && \
	source "$(IDF_PATH)/export.sh" >/dev/null && \
	idf.py -B $(BUILD) build -DDEVICE=$(DEVICE)

# Hardware

.PHONY: flash
flash: build
	source "$(IDF_PATH)/export.sh" && \
	idf.py -B $(BUILD) flash -p $(PORT)

.PHONY: flashmonitor
flashmonitor: build
	source "$(IDF_PATH)/export.sh" && \
	idf.py -B $(BUILD) flash -p $(PORT) monitor

.PHONY: erase
erase:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) erase-flash -p $(PORT)

.PHONY: monitor
monitor:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) monitor -p $(PORT)

# Debugging

.PHONY: openocd
openocd:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) -DDEVICE=$(DEVICE) openocd

.PHONY: gdb
gdb:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) -DDEVICE=$(DEVICE) gdb

.PHONY: gdbgui
gdbgui:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) -DDEVICE=$(DEVICE) gdbgui

.PHONY: gdbtui
gdbtui:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) -DDEVICE=$(DEVICE) gdbtui

# Tools

.PHONY: size
size:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) size

.PHONY: size-components
size-components:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) size-components

.PHONY: size-files
size-files:
	source "$(IDF_PATH)/export.sh" && idf.py -B $(BUILD) size-files

# Formatting

.PHONY: format
format:
	find main/ -iname '*.h' -o -iname '*.c' -o -iname '*.cpp' | xargs clang-format -i

# Badgelink (for Tanmatsu app install)
.PHONY: badgelink
badgelink:
	rm -rf badgelink
	git clone https://github.com/badgeteam/esp32-component-badgelink.git badgelink
	cd badgelink/tools; ./install.sh

.PHONY: install
install: build
	export IDF_TOOLS_PATH="$(IDF_TOOLS_PATH)" && \
	source "$(IDF_PATH)/export.sh" >/dev/null && \
	cd badgelink/tools && ./badgelink.sh appfs upload application "tiny386" 0 ../../$(BUILD)/tiny386.bin

.PHONY: run
run:
	cd badgelink/tools; ./badgelink.sh start application

# Help
.PHONY: help
help:
	@echo "tiny386 ESP32 Build System"
	@echo ""
	@echo "Usage: make [target] [DEVICE=<device>] [PORT=<port>]"
	@echo ""
	@echo "Devices: tanmatsu, jc4880p433, jc3248w535"
	@echo "Default: DEVICE=tanmatsu PORT=/dev/ttyACM0"
	@echo ""
	@echo "Targets:"
	@echo "  prepare      - Initialize SDK and generate lookup tables"
	@echo "  build        - Build firmware for DEVICE"
	@echo "  flash        - Build and flash to device"
	@echo "  flashmonitor - Build, flash and open serial monitor"
	@echo "  monitor      - Open serial monitor"
	@echo "  install      - Install via badgelink (Tanmatsu only)"
	@echo "  clean        - Remove build directory for DEVICE"
	@echo "  fullclean    - Remove all build artifacts"
	@echo "  menuconfig   - Open SDK configuration menu"
	@echo "  help         - Show this help"
