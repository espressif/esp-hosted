# ESP-Hosted-MCU: Espressif SoCs as Communication Co-Processors

## 1 Introduction

ESP-Hosted-MCU is an open-source solution that allows you to use Espressif Chipsets and modules as a communication co-processor. This solution provides wireless connectivity (Wi-Fi and Bluetooth) to the host microprocessor or microcontroller, enabling it to communicate with other devices. Additionally, the user has complete control over the co-processor's resources.

This high-level block diagram shows ESP-Hosted's relationship with the host MCU and slave co-processor.

<img src="docs/images/ESP-Hosted-FG-MCU_design.svg" alt="ESP-Hosted">

For detailed design diagrams in Wi-Fi and Bluetooth, refer to the following design documents:

- [WiFi Design](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/wifi_design.md)
- [Bluetooth Design](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/bluetooth_design.md)

This branch, `feature/esp_as_mcu_host` is dedicated for any host as MCU support. If you are interested in Linux as host, please refer to [`master`](https://github.com/espressif/esp-hosted/blob/master) branch.

## 2 Architecture

##### Hosted Co-Processor
This is an ESP chip that provides Wi-Fi, Bluetooth, and other capabilities. It is also referred as `hosted-slave` interchangeably.

##### Host MCU
This can be any generic microcontroller (MCU). We demonstrate any ESP as host. Using port layer, any host can act as host MCU.

##### Communication
- Host extends the capabilities of the Hosted co-processor through Remote Procedure Calls (RPCs). The Host MCU sends these RPC commands to the Hosted co-processor using a reliable communication bus, like SPI, SDIO, or UART. The Hosted co-processor then handles the RPC and provides the requested functionality to the Host MCU.
- The data (network or Bluetooth) is packaged efficiently at the transport layer to minimize overhead and delays when passing between the Host and co-processor.
- This modular design allows any MCU to be used as the Host, and any ESP chip with Wi-Fi and/or Bluetooth to be used as the Hosted co-processor. The RPC calls can also be extended to provide any function required by the Host, as long as the co-processor can support it.

## 3 Solution Flexibility

- **Any MCU can be the host**
  - You can evaluate ESP as an example host and then port ESP-Hosted to your desired MCU.
- **Any ESP chip can be the co-processor**
  - Any Wi-Fi and/or Bluetooth capable ESP chipset can be chosen as co-processor
  - Choose the co-processor device based on your product requirements. The [ESP Product Selector](https://www.espressif.com/en/products/socs) can help in this.
- **Flexible transport layer (SDIO, SPI, UART)**
  - ESP-Hosted supports various communication interfaces between the host and the co-processor, allowing you to choose the most suitable one for your application.
  - Any other new transport also could be added to the open source code
- **Complete control over co-processor's resources**
  - The user is not limited to just using the co-processor for wireless connectivity. They have complete control over the co-processor's resources, allowing for a more flexible and powerful system.
- **Extensible RPC library**
  - The Remote Procedure Call (RPC) used by ESP-Hosted can be extended to provide any function required by the Host, as long as the co-processor can support it. Currently, the essential [ESP-IDF](https://github.com/espressif/esp-idf) Wi-Fi functions have been implemented.

## 4 Quick Demo with ESP32-P4-Function-EV-Board

Impatient to test? We've got you covered!
The [ESP32-P4-Function-EV-Board](https://www.espressif.com/en/products/socs/esp32-p4) can be used as a host MCU with an on-board [ESP32-C6](https://www.espressif.com/en/products/socs/esp32-c6) as co-processor, already connected via SDIO as transport.
Prerequisite: You need to have an ESP32-P4-Function-EV-Board`

> [!NOTE]
> If you have already set up ESP-IDF (version 5.3 or later), you can skip to [5 Source Code and Dependencies](#5-source-code-and-dependencies).

### 4.1 Set-Up ESP-IDF

- Windows
  - Install and setup ESP-IDF on Windows as documented in the [Standard Setup of Toolchain for Windows](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html).
  - Use the ESP-IDF [Powershell Command Prompt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#using-the-command-prompt) to move to expected

- Linux or MacOS
  - bash
```bash
bash docs/setup_esp_idf__latest_stable__linux_macos.sh
```
  - fish
```fish
fish docs/setup_esp_idf__latest_stable__linux_macos.fish
```

### 4.2 Set-Up P4 with C6
The host, ESP32-P4, lacks native Wi-Fi/Bluetooth support. Our [Quick Demo](docs/esp32_p4_function_ev_board.md) will help you run iperf over P4--SDIO--C6.

### 4.3 Don't Have ESP32-P4-Function-EV-Board?

No worries if you don't have an ESP32-P4. In fact, most users don't. You can choose and use any two ESP chipsets/SoCs/Modules/DevKits. DevKits are convenient to use as they have GPIO headers already in place. From these two ESP chipsets, one would act as host and another as slave/co-processor. However, as these are not connected directly, you would need to manually connect some transport, which is explained later in the section [`Detailed Setup`](#7-detailed-setup).

## 5 Source Code and Dependencies

### 5.1 ESP-Hosted-MCU Source Code

- ESP-Hosted-MCU code can be found at Espressif Registry Component [`esp_hosted` (ESP-Hosted)](https://components.espressif.com/components/espressif/esp_hosted) or GitHub repo at [`ESP-Hosted`](https://github.com/espressif/esp-hosted/tree/feature/esp_as_mcu_host)

- ESP-Hosted repo clone is **not** required if you have ESP as host.
  - Reason: [ESP component manager](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-component-manager.html) automatically clones esp-hosted component while building.
- However, For non-ESP host development, you can clone the repo using command:
```bash
git clone --recurse-submodules --branch feature/esp_as_mcu_host --depth 1 \
https://github.com/espressif/esp-hosted esp_hosted_mcu
```

### 5.2 Dependencies

ESP-Hosted-MCU Solution is dependent on `ESP-IDF`, `esp_wifi_remote` and `protobuf-c`

###### ESP-IDF
  - [`ESP-IDF`](https://github.com/espressif/esp-idf) is the development framework for Espressif SoCs supported on Windows, Linux and macOS
  - ESP-Hosted-MCU solution is based on ESP-IDF as base software. ESP chipsets as host and slave always tried to design such a way that ESP-IDF components are re-used.
  - Although, We totally understand, host MCUs in case of non-ESP chipset may not desire to be dependent on ESP-IDF. The port layer is written to avoid suc dependencies. Some crucial ESP-IDF components could also be just copy-pasted to fast-track the non-ESP host development.

###### Wi-Fi Remote
  - [`esp_wifi_remote`](https://components.espressif.com/components/espressif/esp_wifi_remote) i.e. 'Wi-Fi Remote' is very thin interface made up of ESP-IDF Wi-Fi APIs with empty weak definitions. Real definitions for these APIs are provided by ESP-Hosted-MCU
  - Wi-Fi Remote Code can be found at either [GitHub Repo](https://github.com/espressif/esp-protocols/tree/master/components/esp_wifi_remote) or [Espressif Registry Component](https://components.espressif.com/components/espressif/esp_wifi_remote)

###### Protobuf
  - [`protobuf-c`](https://github.com/protobuf-c/protobuf-c) is data serialization framework provided by Google. RPC messages communicated in host and slave are protobuf encoded.
  - It helps to avoid manual serialization or endien-ness conversion.
  - Provides Flexibility for users to port the ESP-Hosted-MCU RPC framework in any protobuf supported programming language
  - Code is checked-out as submodule at `common/protobuf-c`

##### 5.2.1 How Dependencies Work Together (short explanation)
- RPC Request - Response
  - Wi-Fi Remote is an API layer or interface that provides the standard ESP-IDF Wi-Fi calls to the application (`esp_wifi_init()`, etc.)
  - Wi-Fi Remote forwards the Wi-Fi calls to ESP-Hosted, as ESP-Hosted 'implements' tha APIs provided by Wi-Fi Remote interface.
  - ESP-Hosted host MCU creates RPC requests which are protobuf encoded and sends over the transport (SPI/SDIO etc) to the slave.
  - Slave de-serialize the protobuf RPC request and response send back to host over transport, again with protobuf serialised.
  - Responses received at transport returned to Wi-Fi Remote, which returns the reponses to the calling app at host
  - To the app, it is as if it made a standard ESP-IDF Wi-Fi API call.
- RPC Event
  - Asynchronous Wi-Fi events when subscribed, are sent by slave to host.
  - These events terminate in standard ESP-IDF event loop on the host
- Please note, Only RPC i.e. control packets are serialised. Data Packets are never serialised as they do not need endien conversion.

## 6 Decide the communication bus in between host and slave

The communication bus is required to be setup correctly between host and slave.
We refer this as `transport medium` or simply `transport`.

ESP-Hosted-MCU supports SPI/SDIO/UART transports. User can choose which transport to use. Choosing specific transport depends on factors: high performance, easy and quick to test, number of GPIOs used, or simply co-processor preference

Below is chart for the transport medium comparison.

Legends:

- `FD` : Full duplex communication
- `HD` : Half duplex communication
- `BT` : Bluetooth
- `+2` in column `Num of GPIOs`
  - There are two GPIOs additional applicable for all the transports
  - (1) Co-Processor reset: Host needs one additional pin to connect to `RST`/`EN` pin of co-processor, to reset on bootup
  - (2) Ground: Grounds of both chipsets need to be connected.
  - If you use jumper cable connections, connect as many grounds as possible in between two boards for better noise cancellation.
- `Any_Slave`
  - Co-processor suppored: ESP32, ESP32-C2, ESP32-C3, ESP32-C5, ESP32-C6, ESP32-S2, ESP32-S3
  - Classic ESP32 supports 'Classic BT', 'BLE 4.2' & 'BTDM'
  - Rest all chipsets support BLE only. BLE version supported is 5.0+. Exact bluetooth versions could be refered from [ESP Product Selector Page](https://products.espressif.com/#/product-selector)
- `Dedicated platforms`
  - Bluetooth uses dedicated platform, UART and Wi-Fi uses any other base transport
  - In other platforms, Bluetooth and Wi-Fi re-use same platform and hence use less GPIOs and less complicated
  - This transport combination allows Bluetooth to use dedicated uart transportt with additional 2 or 4 depending on hardware flow control.
- (S) : Sheild box reading
- (O) : Over the air reading
- TBD : To be determined
- iperf : iperf2 with test resukts in mbps

**Host can be any ESP chipset or any non-ESP MCU.**

###### Hosted Transports table

| Transport | Type | Num of GPIOs | Setup with | Co-processor supported | Host Tx iperf | Host Rx iperf | Remarks |
|:---------------:|:-----:|:------------:|:----------------:|:--------------:|:------------:|:-----------:|:--------------------------:|
| Standard SPI | FD | 6 | jumper or PCB | Any_Slave | udp: 24 tcp: 22 | udp: 25 tcp: 22| Simplest solution for quick test |
| Dual SPI | HD | 5 | jumper or PCB | Any_Slave [1] | udp: 32 tcp: 26 (O) | udp: 33 tcp: 25 (O) | Better throughput, but half duplex |
| Quad SPI | HD | 7 | PCB only | Any_Slave [1] | udp: 41 tcp: 29 (O) | udp: 42 tcp: 28 (O) | Due to signal integrity, PCB is mandatory |
| SDIO 1-Bit | HD | 4  | jumper or PCB | ESP32, ESP32-C6 | TBD | TBD | Stepping stone for PCB based SDIO 4-bit |
| SDIO 4-Bit | HD | 6 | PCB only | ESP32, ESP32-C6 | udp: 79.5 tcp: 53.4 (S) | udp: 68.1 tcp: 44 (S) | Highest performance |
| Only BT over UART | FD | 2 or 4 | jumper or PCB | Any_Slave | NA | NA | Dedicated Bluetooth over UART pins |
| UART | FD | 2 | jumper or PCB | Any_Slave | udp: 0.68 tcp: 0.67 (O) | udp: 0.68 tcp: 0.60 (O) | UART dedicated for BT & Wi-Fi [2] |
| Dedicated platforms | FD | Extra 2 or 4 | jumper or PCB | Any_Slave | NA | NA | UART dedicated for BT & Wi-Fi on any other transport |

> [!NOTE]
> - [1] Dual/Quad SPI is not supported on ESP32
>
> - [2] UART is only suitable for low throughput environments

With jumper cables, 'Standard SPI' and 'Dual SPI' solutions are easiest to evaluate, without much of hardware dependencies. SDIO 1-Bit can be tested with jumper cables, but it needs some additional hardware config, such as installation of external pull-up registers.

In case case of dedicated platforms, Blutooth uses standard HCI over UART. In rest of cases, Bluetooth and Wi-Fi uses same transport and hence less GPIOs and less complicated. In shared mode, bluetooth runs as vHCI (multiplexed mode)

## 7 ESP-Hosted-MCU Header

### 7.1 ESP Hosted header

Host and slave always populate below header at the start of every frame, irrespective of actual or dummy data in payload.

| Field          | Type     | Bits | Mandatory? | Description                                                                 |
|----------------|----------|------|------------|-----------------------------------------------------------------------------|
| if_type        | uint8_t  | 4    | M          | Interface type                                                              |
| if_num         | uint8_t  | 4    | M          | Interface number                                                            |
| flags          | uint8_t  | 8    | M          | Flags for additional information                                            |
| len            | uint16_t | 16   | M          | Length of the payload                                                       |
| offset         | uint16_t | 16   | M          | Offset for the payload                                                      |
| checksum       | uint16_t | 16   | M          | Checksum for error detection  (0 if checksum disabled)                      |
| seq_num        | uint16_t | 16   | O          | Sequence number for tracking packets (Useful in debugging)                  |
| throttle_cmd   | uint8_t  | 0 or 2    | O          | Flow control command                                                            |
| reserved2      | uint8_t  | 6 or 8    | M          | Reserved bits                                                               |
| reserved3      | uint8_t  | 8    | M          | Reserved byte (union field)                                                 |
| hci\_pkt\_type or priv\_pkt\_type   | uint8_t  | 8    | M          | Packet type for HCI interface (union field)                                 |

### 7.2 Interface Types

Start of header states which type of frame is being carried.

| Interface Type       | Value | Description                                      |
|----------------------|-------|--------------------------------------------------|
| ESP\_INVALID\_IF       | 0     | Invalid interface                                |
| ESP\_STA\_IF           | 1     | Station frame                                    |
| ESP\_AP\_IF            | 2     | SoftAP frame                                     |
| ESP\_SERIAL\_IF        | 3     | Control frame                                    |
| ESP\_HCI\_IF           | 4     | Bluetooth vHCI frame                            |
| ESP\_PRIV\_IF          | 5     | Private communication between slave and host     |
| ESP\_TEST\_IF          | 6     | Transport throughput test                        |
| ESP\_ETH\_IF           | 7     | Invalid                                          |
| ESP\_MAX\_IF           | 8     | type mentioned in dummy or empty frame           |

## 8 Detailed Setup

Once you decided the transport to use, this section should guide how to set this transport, with hardware connections, configurations and verification. Users can evaluate one transport first and then move to other.

> [!IMPORTANT]
>
> [Design Considerations](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/design_consideration.md) that could be reffered to, before you stick to any transport option. Referring to these consideration would help to get you faster to solution, make your design stable and less error-prone.


Irrespective of transport chosen, following steps are needed, which are step-wise explained in each transport.

1. Set-up the hosted-transport
2. Slave Flashing
  - Slave project creation
  - Slave configuration
  - Slave flashing
  - Slave logs
3. Host flashing
  - Host project integration with ESP-IDF example
  - Host configuration
  - Host flashing
  - Host logs

- [**Standard SPI (Full duplex)**](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/spi_full_duplex.md)

- [**SPI - Dual / Quad Half Duplex**](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/spi_half_duplex.md)

- [**SDIO (1-Bit / 4-Bit)**](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/sdio.md)

- [**UART for Wi-Fi and Bluetooth**](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/uart.md)

## 9 Examples
Check [examples](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/examples) directory for sample applications using ESP-Hosted.
 - `examples/bleprph_host_only_vhci`
   - Bluetooth without needing extra GPIOs

## 10 Troubleshooting

If you encounter issues with using ESP-Hosted, see the following guide:

- [Troubleshooting Guide](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/troubleshooting.md)

## 11 References

- [ESP Product Selector Page](https://products.espressif.com)
- [ESP-IDF Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started)
- [ESP-IDF Wi-Fi API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
- [ESP-IDF Iperf Example](https://github.com/espressif/esp-idf/tree/master/examples/wifi/iperf)
- [ESP-IDF NimBLE](https://github.com/espressif/esp-nimble)
- [ESP Component Registry](https://components.espressif.com)
- [Registry Component: esp\_wifi\_remote](https://components.espressif.com/components/espressif/esp_wifi_remote)
- [Registry Component: esp\_hosted](https://components.espressif.com/components/espressif/esp_hosted)
