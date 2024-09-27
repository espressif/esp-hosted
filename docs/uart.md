# ESP-Hosted UART Operation for Wi-Fi and Bluetooth

Section 6 to 8 covers the complete step-wise setup co-processor and host with UART.

If you wish to skip the theory, you can refer the [Quick Start Guide](#1-quick-start-guide) below. For quick navigation, please unfold the Table of Contents below.

<details>
<summary>Table of Contents</summary>

- [1 Quick Start Guide](#1-quick-start-guide)

- [2 Introduction](#2-introduction)

- [3 Hardware Considerations](#3-hardware-considerations) => [1 GPIO Configuration for UART](#31-gpio-configuration-for-uart) || [2 Extra GPIO Signals Required](#32-extra-gpio-signals-required) || [3 General Hardware Considerations](#33-general-hardware-considerations) || [4 PCB Design](#34-pcb-design) || [5 Advanced Considerations](#35-advanced-considerations)

- [4 Hardware Setup](#4-hardware-setup)

- [5 Set-Up ESP-IDF](#5-set-up-esp-idf) => [Option 1: Installer Way](#option-1-installer-way) || [Option 2: Manual Way](#option-2-manual-way)

- [6 Flashing the Co-processor](#6-flashing-the-co-processor) => [1 Create Co-processor Project](#61-create-co-processor-project) || [2 Co-processor Config](#62-co-processor-config) || [3 Co-processor Build](#63-co-processor-build) || [4 Co-processor Flashing](#64-co-processor-flashing)

- [7 Flashing the Host](#7-flashing-the-host) => [1 Select Example to Run in Hosted Mode](#71-select-example-to-run-in-hosted-mode) || [2 Host Project Component Configuration](#72-host-project-component-configuration) || [3 Menuconfig, Build and Flash Host](#73-menuconfig-build-and-flash-host)

- [8 Testing and Troubleshooting](#8-testing-and-troubleshooting)

- [9 References](#9-references)

</details>

## 1 Quick Start Guide

This section provides a brief overview of how to get started with ESP-Hosted using UART mode.

These sections will guide you through the process of flashing both the co-processor and host devices, setting up the hardware connections, and verifying successful communication.

## 2 Introduction

UART is a low-speed bus that only requires two signal lines to communicate between the host and co-processor.

UART is supported on all ESP devices, and many other MCUs and operating systems. It is quick to bring up and test.

However, UART is a low-speed bus, and not recommended for environments where high network throughput (more than 1 Mbits/s) is required.

> [!NOTE]
> UART here is used to transport both Wi-Fi and Bluetooth data (VHCI). Do not confuse this with the standard Bluetooth over UART implementation (HCI), which does not support Wi-Fi.

## 3 Hardware Considerations

### 3.1 GPIO Configuration for UART

The UART interface can use almost any GPIO pins. For maximum speed and minimal delays, it is recommended to select the SDIO pin configuration that uses the dedicated `IO_MUX` pins.

### 3.2 Extra GPIO Signals Required

Extra GPIO signals are required for UART on Hosted and can be assigned to any free GPIO pins:

- `Reset` signal: an output signal from the host to the co-processor. When asserted, the host resets the co-processor. This is done when ESP-Hosted is started on the host, to synchronise the state of the host and co-processor.

> [!NOTE]
> The `Reset` signal suggested to connect to the `EN` or `RST` pin on the co-processor, It is however configurable to use another GPIO pin.
>
> To configure this, use `idf.py menuconfig` on the co-processor: **Example configuration** ---> **UART Configuration** and set **Slave GPIO pin to reset itself**.

### 3.3 General Hardware Considerations

- Due to UART's low speed, signal integrity is less of a concern compared to SPI or SDIO. However, general rules on signal routing and noise reduction still applies.
- Jumper wires are only suitable for initial testing and prototyping.
- Ensure equal trace lengths for all UART connections, whether using jumper wires or PCB traces.
- Keep wires as short as possible, under 10 cm. Smaller the better.
- Use the lower baud rates like 115200 for evaluation. Once solution verified, optimise the baud rate in increasing steps to max possible value.
- Provide proper power supply for both host and co-processor devices. Lower or incorrect power supplies can cause communication issues & suboptimal performance.
- If possible, use a ground wire between every signal wire to improve signal integrity.
- Connect as many grounds as possible to improve common ground reference and reduce ground noise.

### 3.4 PCB Design

For optimal performance and reliability in production designs:

- Ensure equal trace lengths for all UART signals (Rx, Tx) as much as possible. This practice, known as length matching, is crucial for maintaining signal integrity and reducing timing skew, especially at higher frequencies.
- Use controlled impedance traces for high-speed signals.
- Place bypass capacitors close to the power pins of both the host and co-processor devices.
- Consider using series termination resistors on the clock and data lines to reduce reflections.
- For high-speed designs, use a 4-layer PCB with dedicated power and ground planes.

### 3.5 Advanced Considerations

- Calculate the maximum allowed trace length based on your baud rate and PCB material.
- Consider the capacitive load on the UART signals, especially for longer traces
- For very high-speed designs, consider using differential signaling techniques.
- Implement proper EMI/EMC design techniques to minimize electromagnetic interference.

## 4 Hardware Setup

Setting up the hardware involves connecting the master and co-processor devices via the UART pins and ensuring all extra GPIO signals are properly connected.

Any GPIO pin can be used for ESP-Hosted UART Rx and Tx. But avoid using the ESP assigned UART Tx0 and Rx0 pins. There are for debugging output. (ESP-Hosted uses another UART controller.)

## 5 Set-Up ESP-IDF

Before setting up the ESP-Hosted co-processor & host for UART mode, ensure that ESP-IDF is properly installed and set up on your system.

### Option 1: Installer Way

- **Windows**
  - Install and setup ESP-IDF on Windows as documented in the [Standard Setup of Toolchain for Windows](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html).
  - Use the ESP-IDF [Powershell Command Prompt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#using-the-command-prompt) for subsequent commands.

- **Linux or MacOS**
  - For bash:
    ```bash
    bash docs/setup_esp_idf__latest_stable__linux_macos.sh
    ```
  - For fish:
    ```fish
    fish docs/setup_esp_idf__latest_stable__linux_macos.fish
    ```

### Option 2: Manual Way

Please follow the [ESP-IDF Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for manual installation.

## 6 Flashing the Co-processor

| Supported Co-processor Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-S2 | ESP32-S3 |
| ------------------------------ | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

### 6.1 Create Co-processor Project
1. Create co-processor project possibly outside of ESP-IDF project directory using

   ```bash
   idf.py create-project-from-example "espressif/esp_hosted:slave"
   ```

2. Navigate to the created project directory.

3. Configure the project for your target ESP chip:

   ```bash
   idf.py set-target <target>
   ```
   Replace `<target>` with your specific co-processor ESP chip (e.g., esp32c3, esp32s3).

### 6.2 Co-processor Config
Configure the co-processor project using
```
idf.py menuconfig
```

#### 6.2.1 Transport config
  - Navigate to "Example configuration" -> "Transport layer"
  - Select "UART"

#### 6.2.2 Any other config
  - Optionally, Configure any additional UART-specific settings like TX and Rx GPIOs, baud rate, etc.

###### Generated files
- Generated config files are (1) `sdkconfig` file and (2) internal `sdkconfig.h` file.
- Please note, any manually changes done to these generated files, would not take effect.

###### Defaulting specific config (Optional)
- This is advanced option, so please be careful.
- To mark some config options as default, you can add specific config line in file, `sdkconfig.defaults.<target>`. So whenever next time building, you do not need to re-configure.

### 6.3 Co-processor Build
Build the co-processor project

```
idf.py build
```

### 6.4 Co-processor Flashing

There are two methods to flash the ESP-Hosted co-processor firmware:

##### 6.4.1 Serial Flashing (Initial Setup)

For the initial setup or when OTA is not available, use serial flashing.

Flash the co-processor firmware using
```
idf.py -p <co-processor_serial_port> flash
```

> [!NOTE]
>
> If you are not able to flash the co-processor, there might be a chance that host is not allowing to to do so.
>
> Put host in bootloader mode using following command and then retry flashing the co-processor
>
> ```bash
> esptool.py -p **<host_serial_port>** --before default_reset --after no_reset run
> ```

Monitor the output (optional):
```
idf.py -p <coprocessor_serial_port> monitor
```

##### 6.4.2 Co-processor OTA Flashing (Subsequent Updates)

For subsequent updates, you can re-use ESP-Hosted-MCU transport, as it should be already working. While doing OTA, Complete co-processor firmware image is not needed and only co-processor application partition, 'network_adapter.bin' need to be re-flashed remotely from host.

1. Ensure your co-processor device is connected and communicating with the host with existing ESP-Hosted-MCU.

2. Create a web server
You can re-use your existing web server or create a new locally for testing. Below is example to do it.
  - Make a new directory so that web server can be run into it and navigate into it
  - Create simple local web server using python3

     ```bash
     python3 -m http.server 8080
     ```
3. Copy the co-processor app partition `network_adapter.bin` in the directory where you created the web server.
  - The `network_adapter.bin` can be found in your co-processor project build at `<co-processor_project>/build/network_adapter.bin`

4. Verify if web server is set-up correctly
  - Open link `http://127.0.0.1:8080` in the browser and check if network_adapter.bin is available.
  - Right click and copy the complete URL of this network_adapter.bin and note somewhere.

5. On the **host side**, use the `esp_hosted_ota` function to initiate the OTA update:

   ```c
   #include "esp_hosted_api.h"

   const char* image_url = "http://example.com/path/to/network_adapter.bin"; //web server full url
   esp_err_t ret = esp_hosted_ota(image_url);
   if (ret == ESP_OK) {
       printf("co-processor OTA update failed[%d]\n", ret);
   }
   ```

   This function will download the firmware in chunk by chunk as http client from the specified URL and flash it to the co-processor device through the established transport.
   In above web server example, You can paste the copied url earlier.

6. Monitor the OTA progress through the console output on both the host and co-processor devices.

> [!NOTE]
>
> - The `esp_hosted_ota` function is part of the ESP-Hosted-MCU API and handles the OTA process through the transport layer.
> - Ensure that your host application has web server connectivity to download the firmware file.
> - The co-processor device doesn't need to be connected to the web server for this OTA method.

## 7 Flashing the Host

Host are required to support two-line UART and the required baud rate in their hardware. All ESP chipsets hardware support UART.

| Supported Host Targets  | Any ESP chipset | Any Non-ESP chipset |
| ----------------------- | --------------- | ------------------- |

Non ESP chipset may need to port the porting layer. It is strongly recommanded to evaluate the solution using ESP chipset as host before porting to any non-esp chipset.

### 7.1 Select Example to Run in Hosted Mode

Select an example from the [ESP-IDF examples directory](https://github.com/espressif/esp-idf/tree/master/examples) that you wish to run in ESP-Hosted mode. All Wi-Fi and Bluetooth examples are supported. For simplicity and demonstration purposes, we will use the [ESP-IDF iperf example](https://github.com/espressif/esp-idf/tree/master/examples/wifi/iperf).

### 7.2 Host Project Component Configuration

Now that ESP-IDF is set up, follow these steps to prepare the host:

###### 1. Navigate to the iperf example in your ESP-IDF directory:
   ```
   cd $IDF_PATH/examples/wifi/iperf
   ```

###### 2. Dependency components
   Add the required components to the project's `idf_component.yml` file:
   ```
   idf.py add-dependency "espressif/esp_wifi_remote"
   idf.py add-dependency "espressif/esp_hosted"
   ```

###### 3. Remove conflicting configuration
   Open the `main/idf_component.yml` file and remove/comment the following block if present:
   ```
   # ------- Delete or comment this block ---------
   espressif/esp-extconn:
     version: "~0.1.0"
     rules:
       - if: "target in [esp32p4]"
   # -----------------------------------
   ```
   This step is necessary because esp-extconn and esp-hosted cannot work together.

###### 4. Disable native Wi-Fi if available
   If your host ESP chip already has native Wi-Fi support, disable it by editing the `components/soc/<soc>/include/soc/Kconfig.soc_caps.in` file and changing all `WIFI` related configs to `n`.

    If you happen to have both, host and co-processor as same ESP chipset type (for example two ESP32-C2), note an [additional step](docs/troubleshooting/#1-esp-host-to-evaluate-already-has-native-wi-fi)


### 7.3 Menuconfig, Build and Flash Host

###### 1. High performance configurations
   This is optional step, suggested for high performance applications.

   If using ESP32-P4 as host:
     - Remove the default `sdkconfig.defaults.esp32p4` file.
     - Create a new `sdkconfig.defaults.esp32p4` file with the following content:
     ```
     CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=16
     CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=64
     CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=64
     CONFIG_ESP_WIFI_AMPDU_TX_ENABLED=y
     CONFIG_ESP_WIFI_TX_BA_WIN=32
     CONFIG_ESP_WIFI_AMPDU_RX_ENABLED=y
     CONFIG_ESP_WIFI_RX_BA_WIN=32

     CONFIG_LWIP_TCP_SND_BUF_DEFAULT=65534
     CONFIG_LWIP_TCP_WND_DEFAULT=65534
     CONFIG_LWIP_TCP_RECVMBOX_SIZE=64
     CONFIG_LWIP_UDP_RECVMBOX_SIZE=64
     CONFIG_LWIP_TCPIP_RECVMBOX_SIZE=64

     CONFIG_LWIP_TCP_SACK_OUT=y
     ```

    For other hosts also, you can merge above configs in corresponding `sdkconfig.defaults.esp32XX` file.

###### 2. Set environment for your host ESP chip:

   ```
   idf.py set-target <host_target>
   ```
   Replace `<host_target>` with your specific ESP chip (one of esp32, esp32c2, esp32c3, esp32c5, esp32c6, esp32s2, esp32s3, esp32p4).

###### 3. Flexible Menuconfig configurations

   ```
   idf.py menuconfig
   ```
   ESP-Hosted-MCU host configurations are available under "Component config" -> "ESP-Hosted config"
   1. Select "UART" as the transport layer
   2. Change co-processor chipset to connect to under "Slave chipset to be used"
   3. Optionally, Configure UART-specific settings like
     - UART Tx and Rx GPIOs
     - UART baud rate
     - UART Checksum Enable/Disable (Checksum is recommended to be enabled)

  > [!NOTE]
  > The actual baud rate used is determined by the hardware. Use an oscilloscope or logic analyzer to check the baud rate and its accuracy. If the actual baud rate used to send data drifts by more than a few percent from the expected baud rate, the receiver may not be able to correctly decode the data.

###### 4. Build the project:
   ```
   idf.py build
   ```

###### 5. Flash the firmware:
   ```
   idf.py -p <host_serial_port> flash
   ```

###### 6. Monitor the output:
    ```
    idf.py -p <host_serial_port> monitor
    ```
    - If host was put into bootloader mode earlier, it may need manual reset

## 8 Testing and Troubleshooting

After flashing both the co-processor and host devices, follow these steps to connect and test your ESP-Hosted UART setup:

1. Connect the hardware:
   - Follow the pin assignments for UART as specified in [Hardware Setup](#4-hardware-setup).
   - Ensure all necessary connections are made, including power, ground.

2. Power on both devices.

3. Verify the connection:
   - Check the serial output of both devices for successful initialization messages.
   - Look for messages indicating that the UART transport layer has been established.

4. Logs at both sides:
   - Host:

     ```
     I (465) transport: Attempt connection with slave: retry[0]
     I (468) transport: Reset slave using GPIO[54]
     I (473) os_wrapper_esp: GPIO [54] configured
     I (478) gpio: GPIO[54]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
     I (1650) transport: Received INIT event from ESP32 peripheral
     I (1650) transport: EVENT: 12
     I (1650) transport: EVENT: 11
     I (1652) transport: capabilities: 0x88
     I (1657) transport: Features supported are:
     I (1662) transport:        - BLE only
     I (1666) transport: EVENT: 16
     I (1669) transport: extended capabilities: 0x120
     I (1675) transport: Extended Features supported:
     I (1680) transport:      * WLAN over UART
     I (1684) transport: EVENT: 13
     I (1688) transport: ESP board type is : 13

     I (1693) transport: Base transport is set-up
     ```

   - Co-processor:

     ```
	 I (484) fg_mcu_slave: *********************************************************************
     I (493) fg_mcu_slave:                 ESP-Hosted-MCU Slave FW version :: 0.0.6

     I (503) fg_mcu_slave:                 Transport used :: UART only
     I (512) fg_mcu_slave: *********************************************************************
     I (521) fg_mcu_slave: Supported features are:
     I (526) h_bt: - BT/BLE
     I (529) h_bt:    - BLE only
     I (532) fg_mcu_slave: capabilities: 0x88
     I (537) fg_mcu_slave: Supported extended features are:
     I (543) fg_mcu_slave: - WLAN over UART
     I (547) h_bt: - BT/BLE (extended)
     I (551) h_bt:    - HCI Over UART (VHCI)
     I (556) fg_mcu_slave: extended capabilities: 0x120
	 ```

5. Test basic functionality:
   - The iperf example automatically attempts to connect to the configured Wi-Fi network. Watch the serial output for connection status.
   - If the automatic connection fails, you can manually initiate a Wi-Fi scan and connection:
     ```
     sta_scan
     sta_connect <SSID> <password>
     ```
6. Additional commands to test:
   - Get IP address: `sta_ip`
   - Disconnect from Wi-Fi: `sta_disconnect`
   - Set Wi-Fi mode: `wifi_mode <mode>` (where mode can be 'sta', 'ap', or 'apsta')

7. Advanced iperf testing:
   Once connected, you can run iperf tests:

   | Test Case | Host Command | External STA Command |
   |-----------|--------------|----------------------|
   | UDP Host TX | `iperf -u -c <STA_IP> -t 60 -i 3` | `iperf -u -s -i 3` |
   | UDP Host RX | `iperf -u -s -i 3` | `iperf -u -c <HOST_IP> -t 60 -i 3` |
   | TCP Host TX | `iperf -c <STA_IP> -t 60 -i 3` | `iperf -s -i 3` |
   | TCP Host RX | `iperf -s -i 3` | `iperf -c <HOST_IP> -t 60 -i 3` |

   Note: Replace `<STA_IP>` with the IP address of the external STA, and `<HOST_IP>` with the IP address of the ESP-Hosted device.

8. Troubleshooting:
   - If you encounter issues, refer to section 3.3 for checking the UART connection.
   - Consider using a lower baud rate or checking your [hardware setup](#4-hardware-setup) if you experience communication problems.
   - ESP-Hosted-MCU troubleshooting guide: [docs/troubleshooting.md](docs/troubleshooting.md)

9. Monitoring and debugging:
   - Use the serial monitor on both devices to observe the communication between the host and co-processor.
   - For more detailed debugging, consider using a logic analyzer to examine the UART signals.

## 9 References

- ESP Universal Asynchronous Receiver/Transmitter (UART): https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
