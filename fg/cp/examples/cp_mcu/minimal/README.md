# ESP-Hosted Unified Coprocessor Examples

This directory contains examples demonstrating the unified ESP-Hosted coprocessor architecture that supports both Linux and MCU hosts.

## Architecture Overview

The unified coprocessor automatically selects the appropriate components based on the example you choose to build:

```
┌─────────────────┐    ┌─────────────────┐
│  Linux Example  │    │   MCU Example   │
│                 │    │                 │
│ Uses Linux FG   │    │ Uses MCU        │
│ RPC Extension   │    │ RPC Extension   │
└─────────────────┘    └─────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────┐    ┌─────────────────┐
│esp_hosted_ext_  │    │esp_hosted_ext_  │
│  rpc_linux_fg   │    │   rpc_mcu       │
│                 │    │                 │
│ + Linux RPC     │    │ + MCU RPC       │
│ + Linux Proto   │    │ + MCU Proto     │
└─────────────────┘    └─────────────────┘
         │                       │
         └───────────┬───────────┘
                     ▼
            ┌─────────────────┐
            │  esp_hosted_cp  │
            │                 │
            │ + RPC Hooks     │
            │ + Event System  │
            │ + Transport API │
            └─────────────────┘
                     │
                     ▼
            ┌─────────────────┐
            │esp_hosted_      │
            │  transport_cp   │
            │                 │
            │ + SPI/SDIO/UART │
            │ + GPIO Control  │
            │ + Hardware Mgmt │
            └─────────────────┘
```

## Examples

### Linux Host Examples

**[wifi_linux/](./wifi_linux/)** - Basic WiFi coprocessor for Linux ESP-Hosted-FG hosts
- Uses Linux FG RPC protocol
- Compatible with existing Linux ESP-Hosted infrastructure
- Includes protobuf message handling for Linux hosts

### MCU Host Examples  

**[wifi_mcu/](./wifi_mcu/)** - Basic WiFi coprocessor for MCU ESP-Hosted hosts  
- Uses MCU RPC protocol
- Compatible with ESP-Hosted-MCU infrastructure
- Includes protobuf message handling for MCU hosts
- Provides backward compatibility with `esp_hosted_coprocessor_init()`

## Quick Start

### For Linux Hosts

```bash
cd wifi_linux
idf.py set-target esp32c6
idf.py build flash monitor
```

### For MCU Hosts

```bash
cd wifi_mcu  
idf.py set-target esp32c6
idf.py build flash monitor
```

## Component Selection

The build system automatically pulls in the correct extensions based on your example choice:

- **Linux example** → `esp_hosted_ext_rpc_linux_fg` + `esp_hosted_ext_rpc_linux_fg_pbuf` → `esp_hosted_cp` + `esp_hosted_transport_cp`
- **MCU example** → `esp_hosted_ext_rpc_mcu` + `esp_hosted_ext_rpc_mcu_pbuf` → `esp_hosted_cp` + `esp_hosted_transport_cp`

Transport selection is configured via `idf.py menuconfig` → Transport Configuration.

## Migration Notes

### From esp_hosted (Linux)
- Examples now explicitly target Linux hosts with `_linux` suffix
- Same functionality, now with cleaner component separation
- Backward compatible with existing Linux FG hosts

### From esp_hosted_mcu
- Examples now explicitly target MCU hosts with `_mcu` suffix  
- `esp_hosted_coprocessor_init()` function maintained for compatibility
- Same RPC protocol and protobuf messages
- Backward compatible with existing MCU hosts

## Advanced Features

Both example types support advanced features when enabled in `sdkconfig`:

- **Network Split** - ESP handles some network tasks independently
- **Host Power Save** - Advanced power management coordination
- **Custom RPC** - Application-specific commands and data exchange

See the [extensions examples](../extensions/) for advanced feature demonstrations.