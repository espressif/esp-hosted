CC = gcc

# Compiler and linker flags
CROSS_COMPILE :=
CFLAGS = -Wall -g -fPIC
LDFLAGS = -lpthread -lrt

# Directory paths
DIR_ROOT = $(CURDIR)/../..
DIR_COMMON = $(DIR_ROOT)/common
DIR_COMPONENTS = $(DIR_ROOT)/host/components
DIR_SERIAL = $(DIR_ROOT)/host/virtual_serial_if
DIR_LINUX_PORT = $(DIR_ROOT)/host/linux/port
DIR_CONTROL_LIB = $(DIR_ROOT)/host/control_lib

# Include paths
INCLUDE += -I$(DIR_COMMON)/protobuf-c
INCLUDE += -I$(DIR_COMMON)/include
INCLUDE += -I$(DIR_CONTROL_LIB)/include
INCLUDE += -I$(DIR_CONTROL_LIB)/src/include
INCLUDE += -I$(DIR_SERIAL)/include
INCLUDE += -I$(DIR_COMPONENTS)/include
INCLUDE += -I$(DIR_LINUX_PORT)/include

# Source files for the core library
CORE_SRC = $(DIR_COMMON)/protobuf-c/protobuf-c/protobuf-c.c
CORE_SRC += $(DIR_COMMON)/esp_hosted_config.pb-c.c
CORE_SRC += $(DIR_CONTROL_LIB)/src/ctrl_core.c
CORE_SRC += $(DIR_CONTROL_LIB)/src/ctrl_api.c
CORE_SRC += $(DIR_SERIAL)/src/serial_if.c
CORE_SRC += $(DIR_COMPONENTS)/src/esp_queue.c
CORE_SRC += $(DIR_LINUX_PORT)/src/platform_wrapper.c

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
	
