# ESP-Hosted Custom RPC Example

This example demonstrates how to use the modular ESP-Hosted RPC API to handle custom RPC requests and send events from the ESP coprocessor to the host.

## Overview

The ESP-Hosted framework now provides a clean, modular RPC API that allows applications to:

1. **Register custom RPC handlers** using the extension framework: `esp_hosted_cp_register_extension()`
2. **Send custom events** to the host using `esp_hosted_cp_ext_user_defined_rpc_send_event()`
3. **Initialize custom RPC modules** using `esp_hosted_cp_ext_user_defined_rpc_init()`

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Application   │    │ Network          │    │   Host          │
│   (This Example)│    │ Coprocessor      │    │   (Linux)       │
│                 │    │ Component        │    │                 │
├─────────────────┤    ├──────────────────┤    ├─────────────────┤
│ Register RPC    │───▶│ RPC Bridge       │    │ RPC Client      │
│ Handler         │    │                  │    │                 │
│                 │    │ ┌──────────────┐ │    │                 │
│ Send Events     │───▶│ │ Registered   │ │───▶│ Receive Events  │
│                 │    │ │ Handler      │ │    │                 │
│                 │    │ └──────────────┘ │◀───│ Send Requests   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Key Features

### 1. Modular Design
- The network coprocessor component only provides the RPC infrastructure
- Applications register their own custom handlers
- No hardcoded RPC logic in the component

### 2. Clean API
```c
/* Initialize custom RPC module (includes handler registration) */
esp_err_t esp_hosted_cp_ext_user_defined_rpc_init(void);

/* Send custom events to host */
esp_err_t esp_hosted_cp_ext_user_defined_rpc_send_event(uint32_t custom_event_id, const void *data, size_t data_len);

/* Register extension with core component */
esp_err_t esp_hosted_cp_register_extension(esp_hosted_cp_ext_type_t ext_type, esp_hosted_cp_ext_t *ext);
```

### 3. Example Implementation
This example shows how to:
- Handle echo requests (`CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE`)
- Convert requests to events (`CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT`)
- Send periodic events with counter data
- Provide acknowledgment-only responses (`CUSTOM_RPC_REQ_ID__ONLY_ACK`)

## How to Use

### 1. Build and Flash
```bash
cd esp_hosted_fg/coprocessor/examples/extensions/custom_rpc_msg
idf.py build
idf.py flash monitor
```

### 2. Configure WiFi
Use `idf.py menuconfig` to set your WiFi credentials in "Custom RPC Example Configuration".

### 3. Test RPC Communication
From the host side, you can send RPC requests to test the custom handler:
- Send echo requests to see data echoed back
- Send event conversion requests to see them converted to events
- Observe periodic events sent by the example

### 4. CLI Debugging
If CLI is enabled, you can use commands like:
- `help` - Show available commands
- `mem-dump` - Show memory statistics
- `task-dump` - Show task information
- `reboot` - Restart the device

## Extending the Example

To add your own custom RPC handling:

1. **Define your RPC IDs** in `esp_hosted_custom_rpc.h`
2. **Implement your handler** following the pattern in `custom_rpc_handler()`
3. **Create extension structure** and register using `esp_hosted_cp_register_extension()`
4. **Send events** as needed using `esp_hosted_cp_ext_user_defined_rpc_send_event()`
5. **Use the provided init function** `esp_hosted_cp_ext_user_defined_rpc_init()` for standard setup

## Benefits of This Design

1. **Separation of Concerns**: Core networking is separate from application logic
2. **Reusability**: The network coprocessor component can be used by any application
3. **Flexibility**: Applications can implement any custom RPC protocol they need
4. **ESP-IDF Compliance**: Follows standard ESP-IDF component patterns
5. **Easy Testing**: Each application can have its own RPC behavior

This modular approach makes the ESP-Hosted framework much more maintainable and allows for easy customization without modifying core components. 