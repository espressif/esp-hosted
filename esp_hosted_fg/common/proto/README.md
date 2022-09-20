# About Proto Files

The `esp_hosted_config.proto` file is protobuf file which has messages for command and response to communicate between Host and ESP. Using `esp_hosted_config.proto` file, protobuf generated C files are [esp_hosted_config.pb-c.c](../esp_hosted_config.pb-c.c) and [esp_hosted_config.pb-c.h](../include/esp_hosted_config.pb-c.h).

User can add his own message field in `.proto` file and generate respective C files.

To generate C based protobuf files, run
```
cd <path/to/esp_hosted_fg>/common/proto

protoc-c esp_hosted_config.proto --c_out=.

mv esp_hosted_config.pb-c.c ../

mv esp_hosted_config.pb-c.h ../include/
```

Existing control commands are available for use.

To send an new command
1. Add C function in `host/host_common/commands.c`
2. Create python binding in `host/linux/host_control/python_support/commands_map_py_to_c.py` and its python function in `host/linux/host_control/python_support/commands_lib.py`.
3. Add ESP side C function in `esp/esp_driver/network_adapter/main/slave_commands.c`, respective to python function, to handle added message field.

User can test added functionality using `host/linux/host_control/python_support/test.py`.
