# ESP-Hosted Minimal WiFi Example

| Supported Targets | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6/C61 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

This example demonstrates the minimal WiFi-only coprocessor functionality for ESP-Hosted-FG. It provides a basic network coprocessor that can be controlled from a Linux or MCU host.

## Overview

This is the recommended starting point for most ESP-Hosted-FG deployments. It provides:

- **WiFi Station Mode**: Connect to existing WiFi networks
- **WiFi Access Point Mode**: Create WiFi hotspots
- **Control Interface**: Full control path for WiFi configuration
- **Minimal Footprint**: Only includes WiFi functionality, no Bluetooth

## Features

- WiFi 802.11 b/g/n support
- Station and SoftAP modes
- WPA/WPA2/WPA3 security
- Control path over SPI/SDIO
- OTA firmware updates

## Hardware Setup

Connect your ESP device to the host using one of the supported transports:

- SPI : [SPI Setup Guide](../../../../docs/Linux_based_host/SPI_setup.md)
- SDIO : [SDIO Setup Guide](../../../../docs/Linux_based_host/SDIO_setup.md)

## Building and Flashing

### Prerequisites

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
>
> For Windows users, please set up IDF using [Windows IDF Setup Guide](../../sdk_esp_idf_setup/setup_windows11.md)

### Build Steps

1. **Navigate to the example**:
   ```bash
   cd esp_hosted_fg/coprocessor/examples/minimal/wifi
   ```

2. **Set the target chip**:
   ```bash
   idf.py set-target <chip_target>
   ```

<chip_target> could be one of:
`esp32`, `esp32c2`, `esp32c3`, `esp32c5`, `esp32c6`, `esp32s2`, `esp32s3`

3. **Configure the project** (optional):
   ```bash
   idf.py menuconfig
   ```
   Browse to configuration at `(Top) → Component config → ESP-Hosted FG Coprocessor - Core Config`

   Key configuration options:
   - **Transport**: Choose SPI or SDIO under "ESP-Hosted config"
   - **GPIO pins**: Adjust if using custom wiring

4. **Build the firmware**:
   ```bash
   idf.py build
   ```

5. **Flash and monitor**:
   ```bash
   idf.py flash monitor
   ```