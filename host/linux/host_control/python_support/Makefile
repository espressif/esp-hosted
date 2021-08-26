CC = gcc

CFLAGS = -C -Wall -I$(PWD)/../../../host_common/include/ \
	-I$(PWD)/../../../../common/protobuf-c/ \
	-I$(PWD)/../../../../common/include/

TARGET = commands.so

PWD := $(shell pwd)

COMMON_DIR = $(PWD)/../../../../common
HOST_COMMON_DIR = $(PWD)/../../../host_common
C_SUPPORT_DIR = $(PWD)/../c_support

commands.so:
	$(CC) $(CFLAGS) $(COMMON_DIR)/protobuf-c/protobuf-c/protobuf-c.c \
	$(C_SUPPORT_DIR)/platform_wrapper.c \
	$(COMMON_DIR)/esp_hosted_config.pb-c.c \
	$(HOST_COMMON_DIR)/transport_pserial.c \
	-o $(TARGET) -shared -fPIC $(HOST_COMMON_DIR)/commands.c

clean:
	rm -f $(TARGET) *.o
