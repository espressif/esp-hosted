# About Proto Files

The `esp_hosted_config.proto` file is protobuf file which has messages for command and response to communicate between Host and ESP32. Using `esp_hosted_config.proto` file, protobuf generated C and python files are present in `esp/esp_driver/network_adapter/main` and `host/linux/host_control/python_support`.

User can add his own message field in `.proto` file and generate respective C and python files.

To generate C based protobuf files, run
```
protoc-c esp_hosted_config.proto --c_out=../../esp/esp_driver/network_adapter/main/
```

To generate Python based protobuf files, run
```
protoc esp_hosted_config.proto --python_out=../../host/linux/host_control/python_support/
```

Existing control commands available for use. To send an new command, add python function in `host/linux/host_control/python_support/commands.py`. Similarly add respective C function in `esp/esp_driver/network_adapter/main/slave_commands.c` to handle added message field. User can test added functionality using `host/linux/host_control/python_support/test.py`.
