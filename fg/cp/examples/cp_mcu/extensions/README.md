# ESP-Hosted Self-Contained Extensions Examples

This directory contains examples for the new self-contained ESP-Hosted extensions.

## Available Examples

### 1. network_split_example
Demonstrates the **Network Split Extension** functionality:
- Self-contained protobuf protocol
- Independent protocomm endpoint: `"network_split"`
- DHCP/DNS status management
- Event handling for network status changes

### 2. custom_rpc_example
Demonstrates the **Custom RPC Extension** functionality:
- Self-contained protobuf protocol
- Independent protocomm endpoint: `"custom_rpc"`
- Custom RPC message handling (0x1000, 0x1001, 0x1002)
- Event handling for custom RPC operations

### 3. combined_extensions_example
Demonstrates both extensions working together:
- Shows how to use multiple self-contained extensions
- Independent operation of each extension
- Separate protocomm endpoints for each extension

## ⚠️ **DO NOT USE** Directories

- `custom_rpc_msg_donotuse/`: Old incorrect implementation
- `network_split_donotuse/`: Old incorrect implementation

These directories contain the old plugin registry approach that violated the self-contained extension rule.

## Architecture

### Self-Contained Design
Each extension follows the correct architecture:
```
Extension Structure:
├── esp_hosted_ext_<name>/          # Core extension logic
└── esp_hosted_ext_<name>_pbuf/     # Extension protobuf component
```

### Key Features
- ✅ **Independent**: No dependency on core RPC protobuf
- ✅ **Self-contained**: Own protobuf schemas and serialization
- ✅ **Separate endpoints**: Each extension has its own protocomm endpoint
- ✅ **Modular**: Examples can include/exclude extensions independently
- ✅ **ESP-Hosted MCU compatible**: Mimics esp_hosted_mcu RPC patterns

## Usage

### Building Examples

1. **Single Extension Example**:
```bash
cd network_split_example
idf.py build
```

2. **Combined Extensions Example**:
```bash
cd combined_extensions_example
idf.py build
```

### Adding Extensions to Your Project

1. **Add dependencies** to your CMakeLists.txt:
```cmake
REQUIRES esp_hosted_cp esp_hosted_ext_network_split esp_hosted_ext_custom_rpc
```

2. **Initialize extensions** in your main function:
```c
esp_hosted_ext_network_split_init();
esp_hosted_ext_custom_rpc_init();
```

The extensions will automatically register their protocomm endpoints and handle RPC requests from hosts.