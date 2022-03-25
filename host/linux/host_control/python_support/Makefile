CC = gcc

CROSS_COMPILE :=

CFLAGS = -C -Wall

PWD := $(shell pwd)

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
INCLUDE += -I.

SRC += $(DIR_COMMON)/protobuf-c/protobuf-c/protobuf-c.c
SRC += $(DIR_COMMON)/esp_hosted_config.pb-c.c
SRC += $(DIR_CTRL_LIB)/src/ctrl_core.c
SRC += $(DIR_CTRL_LIB)/src/ctrl_api.c
SRC += $(DIR_SERIAL)/src/serial_if.c
SRC += $(DIR_COMPONENTS)/src/esp_queue.c
SRC += $(DIR_LINUX_PORT)/src/platform_wrapper.c

LINKER = -lrt -lpthread

TARGET = commands.so

commands.so:
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(SRC) $(LINKER) -o $(TARGET) -shared -fPIC

clean:
	rm -f $(TARGET) *.o
