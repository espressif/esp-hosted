CC = gcc

# IMX8mm
# SDKTARGETSYSROOT := /opt/fsl-imx-wayland/5.15-kirkstone/sysroots/armv8a-poky-linux
# CROSS_COMPILE := "/opt/fsl-imx-wayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-"
# CFLAGS = -Wall --sysroot=$(SDKTARGETSYSROOT) -g
# LINKER = --sysroot=$(SDKTARGETSYSROOT) -lpthread -lrt

SDKTARGETSYSROOT :=
CROSS_COMPILE :=
CFLAGS = -Wall -g
LINKER = -lpthread -lrt

# Library search path for replxx - can be customized:
#   Default:              make
#   Custom path:          make REPLXX_LIB_PATH=/custom/path
#   Multiple paths:       make REPLXX_LIB_PATH=/path1:/path2
#   Disable RPATH:        make RPATH_FLAGS=
REPLXX_LIB_PATH ?= /usr/local/lib
RPATH_FLAGS = -Wl,-rpath,$(REPLXX_LIB_PATH)

# Sanitizer options - can be customized:
#   Basic (default):      make SANITIZE=basic
#   Full debugging:       make SANITIZE=full
#   Disabled:             make SANITIZE=none
SANITIZE ?= none

ifeq ($(SANITIZE),full)
    CFLAGS_SANITIZE = -fsanitize=undefined -fsanitize-address-use-after-scope -fsanitize-undefined-trap-on-error -fstack-protector-all -fstack-check -fsanitize=address -fsanitize=pointer-compare -fno-omit-frame-pointer -static-libasan
else ifeq ($(SANITIZE),basic)
    CFLAGS_SANITIZE = -fstack-protector-all -fno-omit-frame-pointer
else ifeq ($(SANITIZE),none)
    CFLAGS_SANITIZE =
else
    CFLAGS_SANITIZE = -fstack-protector-all -fno-omit-frame-pointer
endif

# Root directory
DIR_ROOT = $(CURDIR)/../../../../
LIB_DIR = $(DIR_ROOT)/host/control_lib

# Get the control library path
CONTROL_LIB = $(LIB_DIR)/libesp_hosted_rpc.a

# Directory structure
DIR_COMMON = $(DIR_ROOT)/common
DIR_CTRL_LIB = $(DIR_ROOT)/host/control_lib
DIR_SERIAL = $(DIR_ROOT)/host/virtual_serial_if
DIR_COMPONENTS = $(DIR_ROOT)/host/components
DIR_LINUX_PORT = $(DIR_ROOT)/host/linux/port

# Include directories
INCLUDE += -I$(DIR_COMMON)/protobuf-c
INCLUDE += -I$(DIR_COMMON)/include
INCLUDE += -I$(DIR_CTRL_LIB)/include
INCLUDE += -I$(DIR_CTRL_LIB)/src/include
INCLUDE += -I$(DIR_SERIAL)/include
INCLUDE += -I$(DIR_COMPONENTS)/include
INCLUDE += -I$(DIR_LINUX_PORT)/include
INCLUDE += -I/usr/local/include
INCLUDE += -I.

# Set library path
LIBPATH = -L$(LIB_DIR)
LIBS = -Wl,--whole-archive $(CONTROL_LIB) -Wl,--no-whole-archive

# Check if replxx is installed by testing if the compiler can find the header
# This is more flexible than hardcoding paths and works with:
#   - System paths (/usr/include, /usr/local/include)
#   - Custom include paths added via INCLUDE variable
#   - Cross-compilation toolchain paths
REPLXX_INSTALLED := $(shell echo '\#include <replxx.h>' | $(CROSS_COMPILE)$(CC) -E - $(INCLUDE) >/dev/null 2>&1 && echo "yes" || echo "no")

ifeq ($(REPLXX_INSTALLED), yes)
    BUILD_SHELL := hosted_shell
else
    # Skipping `hosted_shell` silently as rpi_init.sh prints a warning and help how to install replxx
    BUILD_SHELL :=
endif

USR_CUSTOM_RPC_OBJS = app_custom_rpc.o

COMMON_OBJS = test_utils.o nw_helper_func.o $(USR_CUSTOM_RPC_OBJS)

.PHONY: test stress hosted_daemon hosted_shell all clean ensure_libs

all: ensure_libs test stress hosted_daemon $(BUILD_SHELL)

# Ensure control library is built
ensure_libs:
	$(MAKE) -C $(DIR_CTRL_LIB) install

TEST_OBJS = test.o $(COMMON_OBJS)

test: ensure_libs test.out

test.out: $(TEST_OBJS)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(LIBPATH) $^ $(LINKER) $(LIBS) -o $(@)

STRESS_OBJS = stress.o $(COMMON_OBJS)

stress: ensure_libs stress.out

stress.out: $(STRESS_OBJS)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) $(LIBPATH) $^ $(LINKER) $(LIBS) -o $(@)

HOSTED_DAEMON_OBJS = hosted_daemon.o $(COMMON_OBJS)

hosted_daemon: ensure_libs hosted_daemon.out

hosted_daemon.out: $(HOSTED_DAEMON_OBJS)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) $(LIBPATH) $^ $(LINKER) $(LIBS) -o $(@)

hosted_shell: ensure_libs hosted_shell.out

hosted_shell.out: hosted_shell.o $(COMMON_OBJS)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) $(LIBPATH) -o $(@) $^ $(LINKER) $(LIBS) -lreplxx -lstdc++

%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -f *.out *.o
	$(MAKE) -C $(DIR_CTRL_LIB) clean
