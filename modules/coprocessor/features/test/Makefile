REPO_ROOT    := $(realpath $(dir $(lastword $(MAKEFILE_LIST)))/../../../../)
TARGET       := ext_test_bin
SYS_TARGET   := sys_test_bin
REG_TARGET   := registry_test_bin
CC           := gcc
CFLAGS       := -std=c11 -Wall -Wextra -Wpedantic -Wshadow

.PHONY: all clean run run_system run_registry all_tests

all: $(TARGET) $(SYS_TARGET) $(REG_TARGET)

$(TARGET): extension_test.c
	$(CC) $(CFLAGS) -o $@ $<

$(SYS_TARGET): system_test.c
	$(CC) $(CFLAGS) -o $@ $<

$(REG_TARGET): registry_test.c
	$(CC) $(CFLAGS) -o $@ $<

run: $(TARGET)
	./$(TARGET)

run_system: $(SYS_TARGET)
	./$(SYS_TARGET)

run_registry: $(REG_TARGET)
	./$(REG_TARGET)

all_tests: run run_registry run_system

clean:
	rm -f $(TARGET) $(SYS_TARGET) $(REG_TARGET)
