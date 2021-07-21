CC = gcc

CROSS_COMPILE :=

CFLAGS = -C -Wall 

DIR_HOST_COMMON = $(PWD)/../../../host_common
DIR_COMMON = $(PWD)/../../../../common/

INCLUDE += -I$(DIR_HOST_COMMON)/include/
INCLUDE += -I$(DIR_COMMON)/protobuf-c/
INCLUDE += -I$(DIR_COMMON)/include/

SRC += $(DIR_COMMON)/protobuf-c/protobuf-c/protobuf-c.c
SRC += $(PWD)/platform_wrapper.c
SRC += $(DIR_COMMON)/esp_hosted_config.pb-c.c
SRC += $(DIR_HOST_COMMON)/transport_pserial.c
SRC += $(DIR_HOST_COMMON)/commands.c test_api.c

PWD := $(shell pwd)

all: test

test:
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(SRC) $(@).c -o $(@).out

stress:
	$(CROSS_COMPILE)$(CC) $(CFLAGS) $(INCLUDE) $(SRC) $(@).c -o $(@).out

clean:
	rm -f *.out *.o
