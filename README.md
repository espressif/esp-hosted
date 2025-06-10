# ESP-Hosted

**ESP-Hosted** is an open-source solution that enables Espressif SoCs/modules (like ESP32) to act as **wireless communication co-processors** for external host systems.

It allows **host devices** (Linux-based systems or microcontrollers, MCUs) to add Wi-Fi and Bluetooth/BLE capabilities via **standard interfaces** like SPI, SDIO, or UART.


### 🔑 Key Features

* **Flexible Connectivity**: Wi-Fi + Bluetooth/BLE
* **Broad Host Support**: Works with Linux and MCU-based systems
* **Multiple Interfaces**: SPI, SDIO, UART
* **Shared Networking**: ESP and host can share the same IP address
* **Power Efficient**: Low power modes for battery-powered use cases

### 📦 High-Level Architecture

<img src="basic_block_diagram.jpg" alt="Basic Block Diagram" width="600"/>

---

## 🧩 ESP-Hosted Variants

ESP-Hosted is available in three main variants:

### 🔹 [ESP-Hosted-NG (Next Gen)](esp_hosted_ng/README.md)

Best for **Linux hosts** needing standard Wi-Fi and Bluetooth integration:

* Acts as a native 802.11 wireless device
* Configurable via `cfg80211` / `wpa_supplicant`
* Supports `NetworkManager`
* Bluetooth via standard HCI interface

---

### 🔹 [ESP-Hosted-FG (First Gen)](esp_hosted_fg/README.md)

Designed for **Linux hosts**, with custom lightweight RPC-based control:

* Ethernet 802.3 interface
* Wi-Fi configuration via protobuf-based RPC
* Fully customizable APIs
* Bluetooth via standard HCI
* Python or C integration
* ESP maintains network when the host is powered off

---

### 🔹 [ESP-Hosted-MCU](https://github.com/espressif/esp-hosted-mcu)

Optimized for **resource-constrained MCUs**:

* Minimal memory footprint
* Wi-Fi configuration via protobuf-based RPC
* Power-efficient operation
* Ready port of ESP and STM32 as host
* Bluetooth via standard HCI
* ESP stays connected even when the host is in deep sleep or powered off


---

## 📊 Variant Comparison

| Feature                    |           ESP-Hosted-NG          |  ESP-Hosted-FG  |  ESP-Hosted-MCU  |
| :-------------------------------- | :------------------------------: | :-------------: | :--------------: |
| **Target Host**            |               Linux              |   Linux / MCU   |        MCU       |
| **Wi-Fi Configuration**    |            `cfg80211`            |  RPC (protobuf) | RPC (protobuf) |
| **Network Interface**      |           802.11 Wi-Fi           |  802.3 Ethernet |  802.3 Ethernet  |
| **Same IP for ESP & Host** |                 ❌                |        ✅        |         ✅        |
| **Power Management**       |                 ✅                |    :hourglass: Planned  |         ✅        |
| **Wi-Fi Modes**            |              STA, AP             | STA, AP, STA+AP |  STA, AP, STA+AP |
| **Bus Interfaces**         |   SPI, SDIO, UART (and combos)   |       Same      |       Same       |
| **Wi-Fi Security**         |       WPA, WPA2, WPA3, Open      |       Same      |       Same       |
| **Standards**              | 802.11 b/g/n/ax, BLE 4.2/5.0/5.3 |       Same      |       Same       |
| **Supported ESP Chips**    |     ESP32, C2/C3/C6, S2/S3    |   ESP32, C2/C3/C5/C6, S2/S3      |    ESP32, C2/C5/C3/C6, S2/S3   |

---
:warning: Note:
[ESP32-C5 beta](https://github.com/espressif/esp-hosted/tree/feat/esp32c5_ng_beta_support) support has been added please click on link for more info

## 🤔 Choosing the Right Variant

| Use Case                                                         | Recommended Variant |
| ---------------------------------------------------------------- | ------------------- |
| Standard Linux Wi-Fi config (`NetworkManager`, `wpa_supplicant`) | **ESP-Hosted-NG**   |
| Linux with custom/proprietary control over Wi-Fi                 | **ESP-Hosted-FG**   |
| Embedded Linux platforms (e.g. Raspberry Pi, BeagleBone)         | **NG** or **FG**    |
| Minimal resource devices (low RAM/CPU MCUs)                      | **ESP-Hosted-MCU**  |
| Custom networking or duplicate stack (same IP on host & ESP)     | **FG** or **MCU**   |
| IoT use cases requiring both BLE and Wi-Fi                       | **Any**             |
| Need for protocol customization / Deep Packet Inspection         | **FG** or **MCU**   |
| Classic Bluetooth support                                        | **All Variants**    |

---

## 📚 Documentation & Resources

### ESP-Hosted-NG

* 📄 [Documentation](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_ng/README.md)
* 🐞 [Issues](https://github.com/espressif/esp-hosted/issues)
* 📈 [Throughput Benchmarks](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_ng/README.md#5-throughput-performance)

### ESP-Hosted-FG

* 📄 [Documentation](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/README.md)
* 🐞 [Issues](https://github.com/espressif/esp-hosted/issues)
* 📈 [Throughput Benchmarks](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/README.md#5-throughput-performance)

### ESP-Hosted-MCU

* 📄 [Documentation](https://github.com/espressif/esp-hosted-mcu/blob/main/README.md)
* 🐞 [Issues](https://github.com/espressif/esp-hosted-mcu/issues)
* 📈 [Throughput Benchmarks](https://github.com/espressif/esp-hosted-mcu/tree/main?tab=readme-ov-file#hosted-transports-table)
