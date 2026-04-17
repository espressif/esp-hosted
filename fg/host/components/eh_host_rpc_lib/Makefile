CC = gcc

# Compiler and linker flags
CROSS_COMPILE :=
CFLAGS = -Wall -g -fPIC
LDFLAGS = -lpthread -lrt

## Directory paths
MAKEFILE_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
DIR_ROOT := $(realpath $(CURDIR)/../../..)
DIR_COMMON = $(DIR_ROOT)/common
EHRCP_LINUX_PROTOBUF = $(DIR_ROOT)/host/components/esp_hosted_cp_lin_fg_pbuf
PROTOBUF_C = $(EHRCP_LINUX_PROTOBUF)/src/protobuf-c/
DIR_COMPONENTS = $(DIR_ROOT)/host/components
DIR_LINUX_PORT = $(DIR_ROOT)/host/linux/port
DIR_CONTROL_LIB = $(DIR_ROOT)/host/components/esp_hosted_rpc_lib
TRANSPORT_DIR = $(MAKEFILE_DIR)/../../../coprocessor/components/esp_hosted_transport_cp

$(info TRANSPORT_DIR = $(TRANSPORT_DIR))

# Include paths
INCLUDE += -I$(TRANSPORT_DIR)/include
INCLUDE += -I$(DIR_COMMON)/protobuf-c
INCLUDE += -I$(DIR_COMMON)/include
INCLUDE += -I$(DIR_CONTROL_LIB)/include
INCLUDE += -I$(DIR_CONTROL_LIB)/src/include
INCLUDE += -I$(DIR_COMPONENTS)/esp_queue/include
INCLUDE += -I$(DIR_CONTROL_LIB)/port/linux/include
INCLUDE += -I$(EHRCP_LINUX_PROTOBUF)/include
INCLUDE += -I$(PROTOBUF_C)

# Source files for the core library
CORE_SRC = $(PROTOBUF_C)/protobuf-c/protobuf-c.c
CORE_SRC += $(EHRCP_LINUX_PROTOBUF)/src/esp_hosted_config.pb-c.c
CORE_SRC += $(DIR_CONTROL_LIB)/src/ctrl_core.c
CORE_SRC += $(DIR_CONTROL_LIB)/src/ctrl_api.c
CORE_SRC += $(DIR_CONTROL_LIB)/src/serial_if.c
CORE_SRC += $(DIR_COMPONENTS)/esp_queue/src/esp_queue.c
CORE_SRC += $(DIR_CONTROL_LIB)/port/linux/src/platform_wrapper.c

# Target library
LIBCONTROL = libesp_hosted_rpc.so
LIBCONTROL_STATIC = libesp_hosted_rpc.a

.PHONY: all clean shared static

all: shared static

shared: $(LIBCONTROL)

static: $(LIBCONTROL_STATIC)

# Build the shared library
$(LIBCONTROL): $(CORE_SRC)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) -shared -o $@ $^ $(LDFLAGS)

# Build the static library
$(LIBCONTROL_STATIC): $(CORE_SRC:.c=.o)
	ar rcs $@ $^

%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

install: all
	@mkdir -p $(DIR_ROOT)/lib
	@cp -f $(LIBCONTROL) $(LIBCONTROL_STATIC) $(DIR_ROOT)/lib/

clean:
	rm -f $(LIBCONTROL) $(LIBCONTROL_STATIC) $(CORE_SRC:.c=.o) 
