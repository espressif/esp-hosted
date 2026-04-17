# Makefile — esp_hosted_frame unit tests
# Run from this directory:  make
# Or from repo root:        make -C components/common/esp_hosted_frame/test

REPO_ROOT := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../../..)

COMMON_INC := $(REPO_ROOT)/components/common/esp_hosted_common/include
FRAME_INC  := $(REPO_ROOT)/components/common/esp_hosted_frame/include
FRAME_SRC  := $(REPO_ROOT)/components/common/esp_hosted_frame/src/esp_hosted_frame.c

CC      := gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wpedantic -Wshadow \
           -I$(COMMON_INC) -I$(FRAME_INC)
TARGET  := frame_test_bin

.PHONY: all clean run

all: $(TARGET)

$(TARGET): frame_test.c $(FRAME_SRC)
	$(CC) $(CFLAGS) -o $@ $(FRAME_SRC) frame_test.c

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
