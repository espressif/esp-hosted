# ESP-Hosted Protobuf Linux Component

## Overview

The `eh_proto_linux_v1` component provides Protocol Buffers (protobuf) support for the ESP-Hosted framework's Linux host implementation. This component manages the generated protobuf message types and serialization/deserialization logic for RPC communication.

## Features

- Protobuf message type definitions for ESP-Hosted RPC
- Serialization and deserialization support
- Generated message structures for:
  - WiFi Configuration
  - Access Point Management
  - System Commands
  - Event Notifications
  - Custom RPC Messages

## Dependencies

- Protobuf-C library
- ESP-IDF core components
- `esp_hosted_cp`: Core Remote Control Processor component

## Usage

```c
#include "eh_proto_linux_v1.h"

void app_main(void)
{
    // Initialize protobuf support
    ESP_ERROR_CHECK(eh_proto_linux_v1_init());
}
```

## Generated Message Types

The component includes generated protobuf message types for:
- Control Messages
- Scan Results
- Connected Station Lists
- WiFi Configuration Structures
- OTA Update Messages
- Custom RPC Messages

## Protobuf Source

The protobuf definitions are generated from the `esp_hosted_config.proto` file, which defines the communication protocol between the host and ESP coprocessor.

## Initialization and Deinitialization

- `eh_proto_linux_v1_init()`: Prepares protobuf support
- `eh_proto_linux_v1_deinit()`: Cleans up protobuf resources

## Limitations

- Requires compatible protobuf-c version
- Generated from a specific protocol definition
- Platform-specific implementation for Linux host

## License

Apache-2.0 License

