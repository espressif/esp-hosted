REPO_ROOT := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../../..)
TARGET    := registry_test_bin
CC        := gcc
CFLAGS    := -std=c11 -Wall -Wextra -Wpedantic -Wshadow

.PHONY: all clean run

all: $(TARGET)

$(TARGET): registry_test.c
	$(CC) $(CFLAGS) -o $@ $<

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)
