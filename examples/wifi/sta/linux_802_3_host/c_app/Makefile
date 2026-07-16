SHELL := /bin/sh

THIS_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
EH_PATH ?= $(abspath $(THIS_DIR)/../../../../..)
BUILD_DIR ?= $(THIS_DIR)/build
TARGET ?= wifi_sta_host_c_app
CONFIG_TARGET ?= linux

.PHONY: help configure reconfigure defconfig menuconfig save-defconfig build all run clean fullclean show

help:
	@printf '%s\n' \
	'Targets:' \
	'  make configure       - cmake -S . -B build' \
	'  make reconfigure     - alias of configure' \
	'  make defconfig       - alias of configure' \
	'  make menuconfig      - run hosted menuconfig target' \
	'  make save-defconfig  - run hosted save-defconfig target' \
	'  make build           - configure and build $(TARGET)' \
	'  make run             - build and sudo-run $(TARGET)' \
	'  make clean           - remove build/' \
	'  make fullclean       - remove build/, bin/, .nvs/' \
	'  make show            - print important paths' \
	'' \
	'Variables:' \
	'  EH_PATH=/path/to/esp_hosted' \
	'  BUILD_DIR=/abs/path/to/build' \
	'  TARGET=$(TARGET)'

$(THIS_DIR)/.eh.target:
	@printf '%s\n' '$(CONFIG_TARGET)' > '$@'

configure: $(THIS_DIR)/.eh.target
	EH_PATH='$(EH_PATH)' cmake -S '$(THIS_DIR)' -B '$(BUILD_DIR)'

reconfigure: configure

defconfig: configure

menuconfig: configure
	cmake --build '$(BUILD_DIR)' --target menuconfig

save-defconfig: configure
	cmake --build '$(BUILD_DIR)' --target save-defconfig

build all: configure
	cmake --build '$(BUILD_DIR)'

run: build
	@bin_path="$$( \
		if [ -x '$(THIS_DIR)/bin/$(TARGET)' ]; then \
			printf '%s\n' '$(THIS_DIR)/bin/$(TARGET)'; \
		elif [ -x '$(BUILD_DIR)/$(TARGET)' ]; then \
			printf '%s\n' '$(BUILD_DIR)/$(TARGET)'; \
		else \
			find '$(BUILD_DIR)' -type f -name '$(TARGET)' -perm -111 2>/dev/null | head -n 1; \
		fi)"; \
	if [ -z "$$bin_path" ]; then \
		printf '%s\n' 'make: unable to locate built executable $(TARGET)' >&2; \
		exit 2; \
	fi; \
	printf 'running %s\n' "$$bin_path"; \
	exec sudo "$$bin_path"

clean:
	rm -rf '$(BUILD_DIR)'

fullclean:
	rm -rf '$(BUILD_DIR)' '$(THIS_DIR)/bin' '$(THIS_DIR)/.nvs'

show:
	@printf '%s\n' \
	'EH_PATH=$(EH_PATH)' \
	'THIS_DIR=$(THIS_DIR)' \
	'BUILD_DIR=$(BUILD_DIR)' \
	'TARGET=$(TARGET)' \
	'CONFIG_TARGET=$(CONFIG_TARGET)'
