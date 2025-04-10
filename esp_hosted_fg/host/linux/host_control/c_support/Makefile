CC = gcc

# IMX8mm
# SDKTARGETSYSROOT := /opt/fsl-imx-wayland/5.15-kirkstone/sysroots/armv8a-poky-linux
# CROSS_COMPILE := "/opt/fsl-imx-wayland/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-"
# CFLAGS = -C -Wall --sysroot=$(SDKTARGETSYSROOT) -g
# LINKER = --sysroot=$(SDKTARGETSYSROOT) -lpthread -lrt

SDKTARGETSYSROOT :=
CROSS_COMPILE :=
CFLAGS = -C -Wall -g
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



DIR_COMMON = $(PWD)/../../../../common
DIR_CTRL_LIB = $(PWD)/../../../control_lib
DIR_SERIAL = $(PWD)/../../../virtual_serial_if
DIR_COMPONENTS = $(PWD)/../../../components
DIR_LINUX_PORT = $(PWD)/../../port

INCLUDE += -I$(DIR_COMMON)/protobuf-c
INCLUDE += -I$(DIR_COMMON)/include
INCLUDE += -I$(DIR_CTRL_LIB)/include
INCLUDE += -I$(DIR_CTRL_LIB)/src/include
INCLUDE += -I$(DIR_SERIAL)/include
INCLUDE += -I$(DIR_COMPONENTS)/include
INCLUDE += -I$(DIR_LINUX_PORT)/include
INCLUDE += -I/usr/local/include
INCLUDE += -I.

SRC = $(DIR_COMMON)/protobuf-c/protobuf-c/protobuf-c.c
SRC += $(DIR_COMMON)/esp_hosted_config.pb-c.c
SRC += $(DIR_CTRL_LIB)/src/ctrl_core.c
SRC += $(DIR_CTRL_LIB)/src/ctrl_api.c
SRC += $(DIR_SERIAL)/src/serial_if.c
SRC += $(DIR_COMPONENTS)/src/esp_queue.c
SRC += $(DIR_LINUX_PORT)/src/platform_wrapper.c
SRC += nw_helper_func.c test_utils.c app_custom_rpc.c

PWD := $(shell pwd)

# Check if replxx is installed by looking for the header file
REPLXX_INSTALLED := $(shell if [ -f /usr/include/replxx.h ] || [ -f /usr/local/include/replxx.h ]; then echo "yes"; else echo "no"; fi)

ifeq ($(REPLXX_INSTALLED), yes)
    BUILD_SHELL := hosted_shell
else
    BUILD_SHELL :=
    $(warning "replxx library not found. hosted_shell will not be built.")
    $(warning "You can install it from https://github.com/AmokHuginnsson/replxx")
    $(warning "Retry building using 'make hosted_shell' in folder 'host/linux/host_control/c_support'")
endif

all: test stress $(BUILD_SHELL)

test: test.c $(SRC)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(SRC)  $(LINKER) $< -o $(@).out

stress: stress.c $(SRC)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) $(SRC) $(LINKER) $< -o $(@).out -ggdb3 -g

hosted_shell: hosted_shell.c $(SRC)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) -o $(@).out $^ $(LINKER) -lreplxx -lstdc++ $(RPATH_FLAGS)

clean:
	rm -f *.out *.o
