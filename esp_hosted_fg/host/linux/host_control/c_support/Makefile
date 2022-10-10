CC = gcc

CROSS_COMPILE :=

CFLAGS = -C -Wall --sysroot=$(SDKTARGETSYSROOT)

CFLAGS_SANITIZE = -fsanitize=undefined  -fsanitize-address-use-after-scope -fsanitize-undefined-trap-on-error  -fstack-protector-all -fstack-check -fsanitize=address -fsanitize=pointer-compare -fno-omit-frame-pointer -static-libasan

LINKER = -lpthread -lrt

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

PWD := $(shell pwd)

SRC += ./test_utils.c

all: test

test:
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(SRC) $(LINKER) $(@).c -o $(@).out

stress:
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) $(SRC) $(LINKER) $(@).c -o $(@).out -ggdb3 -g

clean:
	rm -f *.out *.o
