# ESP-Hosted Linux Examples

Examples for Linux-based hosts using ESP-Hosted-FG coprocessor.

## Minimal Examples

**[WiFi Example](minimal/wifi/README.md)**
- Basic WiFi coprocessor functionality
- Station and Access Point modes

**[Bluetooth Example](minimal/bt/README.md)**
- Bluetooth coprocessor - BLE and/or Classic Bluetooth
- HCI interface over SPI/SDIO/UART

## Extension Examples

**[Custom RPC Example](extensions/custom_rpc/README.md)**
- Application-specific protocols
- Custom data exchange between host and coprocessor

**[Network Split Examples](extensions/network_split/iperf/README.md)**
- Share single IP address between ESP and host
- Network services while host is in low power mode
- See [Network Split documentation](../../../docs/Linux_based_host/Network_Split.md)

## Hardware Support

| Example | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 |
|---------|-------|----------|----------|----------|----------|----------|----------|
| WiFi | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| BT | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Custom RPC | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Network Split | ✅ | ✅ | ✅ | ❌ | ❌ | ✅ | ✅ |

## Transport Options

- **SPI**: All chips, reliable, moderate throughput
- **SDIO**: ESP32 only, higher throughput
- **UART**: Bluetooth only, direct HCI

## Quick Start

1. Choose an example based on your needs (WiFi/BT)
2. Run `idf.py menuconfig` to configure:
   - Transport interface (SPI/SDIO/UART)
   - GPIO pins
   - WiFi/BT settings
3. Build and flash the firmware
4. Build and load the Linux kernel module
5. Use hosted shell for RPC commands
