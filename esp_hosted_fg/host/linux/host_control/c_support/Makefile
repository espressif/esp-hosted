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

#If your Linux is very constraint environment, comment below line.
CFLAGS_SANITIZE = -fsanitize=undefined  -fsanitize-address-use-after-scope -fsanitize-undefined-trap-on-error  -fstack-protector-all -fstack-check -fsanitize=address -fsanitize=pointer-compare -fno-omit-frame-pointer -static-libasan



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

SRC = $(DIR_COMMON)/protobuf-c/protobuf-c/protobuf-c.c
SRC += $(DIR_COMMON)/esp_hosted_config.pb-c.c
SRC += $(DIR_CTRL_LIB)/src/ctrl_core.c
SRC += $(DIR_CTRL_LIB)/src/ctrl_api.c
SRC += $(DIR_SERIAL)/src/serial_if.c
SRC += $(DIR_COMPONENTS)/src/esp_queue.c
SRC += $(DIR_LINUX_PORT)/src/platform_wrapper.c

PWD := $(shell pwd)

all: test stress

test: test.c test_utils.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(SRC) test_utils.c $(LINKER) $< -o $(@).out

stress: stress.c test_utils.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(CFLAGS_SANITIZE) $(INCLUDE) $(SRC) test_utils.c $(LINKER) $< -o $(@).out -ggdb3 -g

clean:
	rm -f *.out *.o
