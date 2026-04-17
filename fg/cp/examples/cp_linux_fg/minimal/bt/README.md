# ESP-Hosted Minimal Bluetooth Example

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

ESP32-H2 support is upcoming for Bluetooth-only functionality.

This example demonstrates the minimal Bluetooth-only coprocessor functionality for ESP-Hosted-FG.

## Overview

This example showcases minimal Bluetooth connectivity implementation. It initializes the Bluetooth controller and exposes it through the ESP-Hosted communication bus (SPI/SDIO).

The Bluetooth stack on Linux systems (such as BlueZ) interacts with this controller using hosted HCI. The Bluetooth capabilities vary depending on the ESP chip used, allowing users to select the appropriate chip for their requirements.

- **Bluetooth Low Energy (BLE)**: All supported chips (BLE versions 4.2/5.x depend on the ESP chip)
- **Classic Bluetooth**: ESP32 only (BR/EDR)
- **Standard HCI Interface**: Standard HCI over UART (RPC is optional over SPI/SDIO)
- **Hosted-HCI Interface**: RPC, Wi-Fi, and Bluetooth are multiplexed over the same SPI/SDIO communication bus (no UART required)

## Features

### Bluetooth Support by Chip
- **ESP32**: BLE 4.2 + Classic Bluetooth (BR/EDR)
- **ESP32-C2/C3/S3**: BLE 4.2 and 5.0
- **ESP32-C5/C6**: BLE 5.3

### Transport Options
- **SPI**: Bluetooth HCI over SPI
- **SDIO**: Bluetooth HCI over SDIO (ESP32 only)
- **UART**: Direct UART HCI (recommended for Bluetooth-only applications)

## Bluetooth Implementation Variations

### 1. Standard HCI over UART

The base Linux Bluetooth driver supports running the Bluetooth stack directly over UART. In this configuration, RPC channel setup using SPI/SDIO transport is entirely optional.

This is a pass-through mode that doesn't involve ESP-Hosted. Please follow the [Bluetooth Passthrough mode over UART](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/docs/Linux_based_host/UART_setup.md) configuration. Once configured, the ESP will expose the Bluetooth controller directly over its UART pins, bypassing ESP-Hosted entirely.

#### 1.1 [Option 1]: No RPC Channel Required

If you don't need RPC communication between the coprocessor and host, modify `app_main.c`:
Remove this line:
```c
ESP_ERROR_CHECK(esp_hosted_ext_rpc_linux_fg_init());
```

This disables the Linux FG RPC extension, leaving only the core coprocessor functionality and transport layer active.

#### 1.2 [Option 2]: RPC Channel Required

The ESP-Hosted RPC channel is established in this case, requiring either SPI or SDIO transport. This example sets up RPC by default while Bluetooth still uses standard HCI over UART.

**Advantages of Standard HCI over UART:**
- ESP-Hosted can be bypassed for UART communication
- Standard HCI exposure makes coprocessor or host easily replaceable without breaking functionality

**Disadvantages of Standard HCI over UART:**
- Additional processing overhead on the coprocessor when RPC channel is needed (ESP must process both UART and RPC channels)
- When RPC is required, two buses must be configured, increasing connections without adding user-visible functionality

#### Setting up Standard HCI over UART

**To use standard HCI over UART, refer to the [UART Setup Guide](../../../../docs/Linux_based_host/UART_setup.md).** Note that the current example doesn't use this configuration but can be easily modified using the additional configuration steps in the UART Setup Guide.

Complete building and flashing instructions are detailed in the UART Setup Guide.

### 2. Hosted HCI over SPI/SDIO (Recommended - Used in This Example)

Since ESP-Hosted transport setup involves configuring SPI or SDIO for RPC communication, standard Bluetooth HCI messages are wrapped with ESP-Hosted headers and interface type identifiers. The interface type indicates the message type in the payload.

In this implementation, all Wi-Fi, RPC, and Bluetooth frames are multiplexed on the same communication bus using interface type identification. This example demonstrates this variation as it's most intuitive for Bluetooth coprocessor applications.

**Advantages of Hosted HCI over SPI/SDIO:**
- Less complex implementation
- Fewer buses required for all combined features
- Fewer GPIO pins needed
- Effortless upgrade to new ESP chip targets with ESP-Hosted firmware

**Disadvantages of Hosted HCI over SPI/SDIO:**
- Hosted header required for multiplexing standard HCI messages
- Non-ESP coprocessor replacement requires additional code to strip ESP-Hosted headers

#### 2.1. Hardware Setup

For detailed wiring diagrams, see:
- [SPI Setup Guide](../../../../docs/Linux_based_host/SPI_setup.md)
- [SDIO Setup Guide](../../../../docs/Linux_based_host/SDIO_setup.md)

#### 2.2. Building and Flashing

##### Prerequisites

1. **ESP-IDF Setup**: Ensure ESP-IDF is installed and configured
   ```bash
   cd esp_hosted_fg/coprocessor/sdk_esp_idf_setup
   ./setup-idf.sh
   ```

2. **Set ESP-IDF Environment**:
   ```bash
   source esp-idf/export.sh
   ```

> [!TIP]
> For Windows users, please set up IDF using the [Windows IDF Setup Guide](../../sdk_esp_idf_setup/setup_windows11.md)

##### Build Steps

1. **Navigate to the example**:
   ```bash
   cd esp_hosted_fg/coprocessor/examples/minimal/bt
   ```

2. **Set the target chip**:
   ```bash
   idf.py set-target <chip_target>
   ```
   `<chip_target>` options: `esp32`, `esp32c2`, `esp32c3`, `esp32c5`, `esp32c6`, `esp32s3`

3. **Configure the project**:
   ```bash
   idf.py menuconfig
   ```
   
   Navigate to: `(Top) → Component config → ESP-Hosted FG Coprocessor - Core Config`

   Optional configurations:
   - **RPC Transport**: Choose SPI or SDIO under `(Top) → Component config → ESP-Hosted FG Coprocessor - Core Config → Transport layer`
   - **GPIO pins**: Adjust for custom wiring
   - **Bluetooth**: Customize under `(Top) → Component config → Bluetooth`

> [!NOTE]
> If using Bluetooth over UART, RPC over SPI/SDIO is optional. For Standard HCI over UART implementation, additional configuration steps are required.

4. **Build the firmware**:
   ```bash
   idf.py build
   ```

5. **Flash and monitor**:
   ```bash
   idf.py flash monitor
   ```

## Next Steps
1. Once you load the host side kernel module for same transport, the bluetooth should be automatically be set up at host bluetooth stack (BlueZ most likely).
```
hciconfig -a
```
Please note `hciconfig` command showcased as it is very commonly used till date. **However**, hciconfig command itself is obsolete by BlueZ. you can use `bluetoothctl show` and `bluetoothctl` shell for further bluetooth interaction.