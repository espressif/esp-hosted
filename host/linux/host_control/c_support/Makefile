CC = gcc

CFLAGS = -C -Wall -I$(PWD)/../../../host_common/include/ -I$(PWD)/../../../../common/protobuf-c/ -I$(PWD)/../../../../common/include/

SRC = $(PWD)/../../../../common/protobuf-c/protobuf-c/protobuf-c.c $(PWD)/platform_wrapper.c $(PWD)/../../../../common/esp_hosted_config.pb-c.c $(PWD)/../../../host_common/transport_pserial.c $(PWD)/../../../host_common/commands.c test_api.c

PWD := $(shell pwd)

all: test

test:
	$(CC) $(CFLAGS) $(SRC) $(@).c -o $(@).out

stress:
	$(CC) $(CFLAGS) $(SRC) $(@).c -o $(@).out

clean:
	rm -f *.out *.o
