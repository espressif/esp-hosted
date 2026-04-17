CC = gcc

CFLAGS = -Wall -fPIC
CROSS_COMPILE :=

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

DIR_ROOT = $(CURDIR)/../../../../
LIB_DIR = $(DIR_ROOT)/host/components/esp_hosted_rpc_lib
CONTROL_LIB = $(LIB_DIR)/libesp_hosted_rpc.a

DIR_COMMON = $(DIR_ROOT)/common
DIR_CTRL_LIB = $(DIR_ROOT)/host/components/esp_hosted_rpc_lib
DIR_SERIAL = $(DIR_ROOT)/host/virtual_serial_if
DIR_COMPONENTS = $(DIR_ROOT)/host/components
DIR_LINUX_PORT = $(DIR_ROOT)/host/linux/port

INCLUDE += -I$(DIR_COMMON)/protobuf-c
INCLUDE += -I$(DIR_COMMON)/include
INCLUDE += -I$(DIR_CTRL_LIB)/include
INCLUDE += -I$(DIR_CTRL_LIB)/src/include
INCLUDE += -I$(DIR_SERIAL)/include
INCLUDE += -I$(DIR_COMPONENTS)/esp_queue/include
INCLUDE += -I$(DIR_CTRL_LIB)/port/linux/include
INCLUDE += -I.

LIBPATH = -L$(LIB_DIR)
LIBS = -Wl,--whole-archive $(CONTROL_LIB) -Wl,--no-whole-archive
LINKER = -lrt -lpthread

TARGET = commands.so

.PHONY: all clean ensure_libs

all: ensure_libs $(TARGET)

ensure_libs:
	@if [ ! -f $(CONTROL_LIB) ]; then \
		echo "Building control library first..."; \
		$(MAKE) -C $(DIR_CTRL_LIB) install; \
	fi

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -shared -o $@ $^ $(LIBPATH) $(LIBS) $(LINKER)

clean:
	rm -f $(TARGET) *.o
	rm -rf __pycache__/  py_parse/__pycache__/
	$(MAKE) -C $(DIR_CTRL_LIB) clean
