# ESP-Hosted SDIO Operation

Sections 3 below covers the hardware requirements like external pull-up requirement, possible efuse burning for co-processor and other hardware aspects to consider for SDIO.

Section 4 to 8 covers the complete step-wise setup co-processor and host with SDIO, for 1-bit and 4-bit SDIO.

If you wish to skip the theory, you can refer the [Quick Start Guide](#1-quick-start-guide) below. For quick navigation, please unfold the Table of Contents below.

<details>
<summary>Table of Contents</summary>
1. [Quick Start Guide](#1-quick-start-guide)

2. [Introduction](#2-introduction)

3. [Hardware Considerations](#3-hardware-considerations) || [3.1 General Considerations](#31-general-considerations) || [3.2 Pull-up Resistors](#32-pull-up-resistors) || [3.3 Voltage Levels & eFuse burning](#33-voltage-levels--efuse-burning) || [3.4 Jumper Wires](#34-jumper-wires) || [3.5 PCB Design](#35-pcb-design) || [3.6 Advanced Considerations](#36-advanced-considerations) || [3.7 Testing Connections](#37-testing-connections)

4. [Hardware Setup](#4-hardware-setup)

5. [Set-Up ESP-IDF](#5-set-up-esp-idf)

6. [Flashing the Co-processor](#6-flashing-the-co-processor) || [6.1 Create Co-processor Project](#61-create-co-processor-project) || [6.2 Co-processor Config](#62-co-processor-config) || [6.3 Co-processor Build](#63-co-processor-build) || [6.4 Co-processor Flashing](#64-co-processor-flashing)

7. [Flashing the Host](#7-flashing-the-host) || [7.1 Select Example to Run in Hosted Mode](#71-select-example-to-run-in-hosted-mode) || [7.2 Host Project Component Configuration](#72-host-project-component-configuration) || [7.3 Menuconfig, Build and Flash Host](#73-menuconfig-build-and-flash-host)

8. [Testing and Troubleshooting](#8-testing-and-troubleshooting)

9. [References](#9-references)

</details>

## 1 Quick Start Guide

This section provides a brief overview of how to get started with ESP-Hosted using SDIO mode. For detailed instructions on each step, please refer to the following sections:

- [4. Hardware Setup](#4-hardware-setup)
- [5. Set-Up ESP-IDF](#5-set-up-esp-idf)
- [6. Flashing the Co-processor](#6-flashing-the-co-processor)
- [7. Flashing the Host](#7-flashing-the-host)
- [8. Testing and Troubleshooting](#8-testing-and-troubleshooting)

These sections will guide you through the process of flashing both the co-processor and host devices, setting up the hardware connections, and verifying successful communication.

## 2 Introduction

SDIO is a high-speed bus that uses the same SDMMC hardware protocol used for SD Cards, but with its own set of commands for communicating with SDIO aware peripherals.

> [!NOTE]
> Only some ESP32 chips support the SDIO Protocol:
>
> A. SDIO as Slave (Co-processor): ESP32, ESP32-C6 \
> B. SDIO as Master: ESP32, ESP32-S3, ESP32-P4


## 3 Hardware Considerations

### 3.1 GPIO Configuration for SDIO

The SDIO interface can use almost any GPIO pins. For maximum speed and minimal delays, it is recommended to select the SDIO pin configuration that uses the dedicated `IO_MUX` pins. Hardware connections in later sections use `IO_MUX` pins, as much as possible.
ESP32 only supports `IO_MUX` pins for SDIO. other chips may support other flexible pins using GPIO_Matrix, with small performance penalty.

### 3.2 Extra GPIO Signals Required

Extra GPIO signals are required for SDIO on Hosted and can be assigned to any free GPIO pins:

- `Reset` signal: an output signal from the host to the co-processor. When asserted, the host resets the co-processor. This is done when ESP-Hosted is started on the host, to synchronise the state of the host and co-processor.

> [!NOTE]
> The `Reset` signal suggested to connect to the `EN` or `RST` pin on the co-processor, It is however configurable to use another GPIO pin.
>
> To configure this, use `idf.py menuconfig` on the co-processor: **Example configuration** ---> **SDIO Configuration** ---> **Host SDIO GPIOs** and set **Slave GPIO pin to reset itself**.


### 3.3 General Hardware Considerations

- For SDIO, signal integrity is crucial, hence jumper wires are not recommended.
- Jumper wires are only suitable for initial testing and prototyping.
- If you wish, you can test SDIO 1-Bit mode using jumper cables, only for initial testing and prototyping. Pull-Ups are still mandatory for all, [CMD, DAT0, DAT1, DAT2, DAT3] irrespective how do you connect, using jumpers or PCB.
- Ensure equal trace lengths for all SDIO connections, whether using jumper wires or PCB traces.
- Very strict requirement, to keep wires as short as possible, under 5 cm. Smaller the better.
- Use the lower clock frequency like 5 MHz for evaluation. Once solution verified, optimise the clock frequency in increasing steps to max possible value. Max SDIO host clock frequency that all SDIO co-processors can work is upto 50 MHz.
- Provide proper power supply for both host and co-processor devices. Lower or incorrect power supplies can cause communication issues & suboptimal performance.

### 3.4 Pull-up Resistors
- SDIO requires external pull-up resistor (51 kOhm recommended) and clean signals for proper operation.
- For this reason, it is not recommended to use jumper cables. Use PCB traces to connect between a Hosted Master and Co-processor.
- For full requirements, refer to ESP-IDF SDIO pull-up resistor requirements at [Pull-Up Requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html).

### 3.5 Voltage Levels & eFuse burning
- SDIO expects all signals to be at 3.3V level. If you are using level shifter, ensure that the level shifter output is set to 3.3V.
- If you use classic ESP32, there is good chance that you would need to burn the eFuse.
- eFuse burning is one time and **non reversible process**. You may brick your device, if burn the eFuse incorrectly.
- Please check full documentation at [eFuse burning](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html) If your chip is listed explicitly, not to burn eFuse, you can ignore this.
- This document covers below issues and their solutions:
  - External pull-ups to be used on: CMD, DAT0, DAT1, DAT2, DAT3, with 51K Ohm recommended, irrespective of jumpers or PCB.
  - Bootstrapping pin and DAT2 voltage issues and solution of eFuse burning, with complete procedure.


### 3.6 Jumper Wires (only for SDIO 1-Bit mode)

- External Pull-ups mandatory for CMD, DAT0, DAT1, DAT2, DAT3 of 51 kOhm.
- Smaller the better, strictly under 5 cm. All equal length.
- Use high-quality, low-capacitance jumper wires.
- Arrange wires to minimize crosstalk, especially for clock and data lines.
- Possibly, use twisted pairs for clock and data lines to reduce electromagnetic interference.
- If possible, use a ground wire between every signal wire to improve signal integrity.
- Connect as many grounds as possible to improve common ground reference and reduce ground noise.

### 3.7 PCB Design

For optimal performance and reliability in production designs:

- Ensure equal trace lengths for all SDIO signals (CLK, CMD, DAT0, DAT1, DAT2, DAT3) as much as possible. This practice, known as length matching, is crucial for maintaining signal integrity and reducing timing skew, especially at higher frequencies.
- If perfect length matching is not possible, prioritize matching the clock (CLK) trace length with the data lines.
- Use controlled impedance traces for high-speed signals.
- Place bypass capacitors close to the power pins of both the host and co-processor devices.
- Consider using series termination resistors on the clock and data lines to reduce reflections.
- For high-speed designs, use a 4-layer PCB with dedicated power and ground planes.

### 3.8 Advanced Considerations

- Calculate the maximum allowed trace length based on your clock frequency and PCB material.
- Consider the capacitive load on the SDIO bus, especially for longer traces
- For very high-speed designs, consider using differential signaling techniques.
- Implement proper EMI/EMC design techniques to minimize electromagnetic interference.

## 4 Hardware Setup

Setting up the hardware involves connecting the master and co-processor devices via the SDIO pins and ensuring all extra GPIO signals are properly connected. Below is the table of connections for the SDIO setup between a host ESP chipset and another ESP chipset as co-processor:



### Host connections
| Signal      | ESP32 | ESP32-S3 | ESP32-P4-Function-EV-Board |
|-------------|-------|----------|----------|
| CLK         | 14    | 19       | 18       |
| Reset Out   | 5     | 42       | 54       |
| CMD         | 15+[ext-pull-up](#34-pull-up-resistors)    | 47+[ext-pull-up](#34-pull-up-resistors)       | 19+[ext-pull-up](#34-pull-up-resistors)       |
| DAT0        | 2+[ext-pull-up](#34-pull-up-resistors)     | 13+[ext-pull-up](#34-pull-up-resistors)       | 14+[ext-pull-up](#34-pull-up-resistors)       |
| DAT1        | 4+[ext-pull-up](#34-pull-up-resistors)     | 35+[ext-pull-up](#34-pull-up-resistors)       | 15+[ext-pull-up](#34-pull-up-resistors)       |
| DAT2        | 12+[ext-pull-up](#34-pull-up-resistors)    | 20+[ext-pull-up](#34-pull-up-resistors)       | 16+[ext-pull-up](#34-pull-up-resistors)         |
| DAT3        | 13+[ext-pull-up](#34-pull-up-resistors)    | 9+[ext-pull-up](#34-pull-up-resistors)        | 17+[ext-pull-up](#34-pull-up-resistors)       |



### Co-processor connections

| Signal      | ESP32 | ESP32-C6 |
|-------------|-------|----------|
| CLK         | 14    | 19       |
| CMD         | 15    | 18       |
| DAT0        | 2     | 20       |
| DAT1        | 4     | 21       |
| DAT2        | 12    | 22       |
| DAT3        | 13    | 23       |
| Reset In    | EN    | EN/RST   |


> [!NOTE]
> 
> A. Try to use IO_MUX pins from the datasheet for optimal performance on both sides. \
> B. These GPIO assignments are based on default Kconfig configurations. You can modify these in the menuconfig for both host and co-processor if needed. \
> C. Once ported, any other host with standard SDIO can be used. \
> D. ESP32, ESP32-S3, and ESP32-P4 can be used as hosts; ESP32 and ESP32-C6 can be used as co-processors in SDIO mode. \
> E. External pull-ups are mandatory

## 5 Set-Up ESP-IDF

Before setting up the ESP-Hosted co-processor & host for SDIO mode, ensure that ESP-IDF is properly installed and set up on your system.

### 5.1 Installer Way

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

### 5.2 Manual Way

Please follow the [ESP-IDF Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for manual installation.

## 6. Flashing the Co-processor

| Supported Co-processor Targets | ESP32 | ESP32-C6 |
| ------------------------------ | ----- | -------- |

There are four steps to flash the ESP-Hosted co-processor firmware:

### 6.1 Create Co-processor Project

1. Navigate to the directory where you want to create the co-processor project.
2. Use the following command to create a new project:
   ```bash
   idf.py create-project <project_name>
   ```
   Replace `<project_name>` with your desired project name.

### 6.2 Co-processor Config

1. Navigate to the project directory:
   ```bash
   cd <project_name>
   ```
2. Configure the project:
   ```bash
   idf.py menuconfig
   ```

#### 6.2.1 Transport config
  - Navigate to "Example configuration" -> "Transport layer"
  - Select "SDIO"

#### 6.2.2 Any other config
  - Optionally, Configure any additional SDIO-specific settings like co-processor GPIOs, SDIO Mode, SDIO timing,etc.

###### Generated files
- Generated config files are (1) `sdkconfig` file and (2) internal `sdkconfig.h` file.
- Please note, any manually changes done to these generated files, would not take effect.

###### Defaulting specific config (Optional)
- This is advanced option, so please be careful.
- To mark some config options as default, you can add specific config line in file, `sdkconfig.defaults.<target>`. So whenever next time building, you do not need to re-configure.

### 6.3 Co-processor Build

1. Build the project:
   ```bash
   idf.py build
   ```

### 6.4 Co-processor Flashing

There are two methods to flash the ESP-Hosted co-processor firmware:

#### 6.4.1 Serial Flashing (Initial Setup)

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
idf.py -p <PORT> monitor
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
> A. The `esp_hosted_ota` function is part of the ESP-Hosted-MCU API and handles the OTA process through the transport layer. \
> B. Ensure that your host application has web server connectivity to download the firmware file. \
> C. The co-processor device doesn't need to be connected to the web server for this OTA method.

## 7 Flashing the Host

| Supported Host Targets  | Any ESP chipset | Any Non-ESP chipset |
| ----------------------- | --------------- | ------------------- |

Any host having SDIO master can be used as host. Please make sure the hardware configurations, like external pull-ups are installed correctly. Tthe voltage at SDIO pins is expected to be 3v3 volts. 
- ESP chipsets as SDIO master
  - ESP as host could be one of ESP32, ESP32-S3, ESP32-P4.
  - For ESP32 as host, may need additional **eFuse burning** for voltage correction on one of data pin. ESP32-S3 and ESP32-P4 does **not** need this.
- Non ESP SDIO Master
  - Any other host having SDIO master can be used as host. Please make sure the hardware configurations, like ([external Pull-up Resistors](#42-pull-up-resistors)) are installed correctly. Tthe voltage at SDIO pins is expected to be 3v3 volts.
- Pull-ups required for CMD, DAT0, DAT1, DAT2, DAT3 lines (for both 1-Bit and 4-Bit SDIO)
- eFuse burning may be required for classic ESP32.
- Pull-Up and eFuse burning is detailed in [(3) Hardware Considerations](#3-hardware-considerations)

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

##### 1. High performance configurations
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
   Replace `<host_target>` with your specific ESP chip (e.g., esp32, esp32s3, esp32p4).
   
###### 3. Flexible Menuconfig configurations

   ```
   idf.py menuconfig
   ```
   ESP-Hosted-MCU host configurations are available under "Component config" -> "ESP-Hosted config"
   1. Select "SDIO" as the transport layer
   2. Change co-processor chipset currently in-use under, "Slave chipset to be used"
   3. Change SDIO Bus Width to 1-bit or 4-bit based on the co-processor using "Hosted SDIO Configuration" -> "SDIO Bus Width"
   4. Optionally, configure SDIO-specific settings like:

   - SDIO Host GPIO Pins

   -  Lower SDIO Clock Speed
      You can use a lower clock speed to verify the connections. Start with a clock speed between 400 kHz to 20 MHz.
      To configure this, use `Menuconfig` on the Host: **Component config** ---> **ESP-Hosted config** ---> **Hosted SDIO Configuration** and set **SDIO Clock Freq (in kHz)**.
    > [!NOTE]
    > 
    > The actual clock frequency used is determined by the hardware. Use an oscilloscope or logic analyzer to check the clock frequency.

   - Using 1-bit SDIO Mode
     By default, SDIO operates in 4-Bit mode.
     You can set the SDIO Bus Width to 1-Bit. In 1-Bit mode, only `DAT0` and `DAT1` signals are used for data and are less affected by noise on the signal lines. This can help you verify that the SDIO protocol is working at the logical level, if you have issues getting 4-Bit SDIO to work on your prototype board.
     
     To configure this, use `Menuconfig` on the Host: **Component config** ---> **ESP-Hosted config** ---> **Hosted SDIO Configuration** ---> **SDIO Bus Width** to **1 Bit**.

   - SDIO Mode
     Packet or Streaming mode could be used, but co-processor has to use same SDIO mode used.

> [!NOTE]

> Pull-ups are still required on `DAT2` and `DAT3` lines to prevent
> the SDIO slave from going into SPI mode upon startup.

After confirming the functionality of the 1-Bit SDIO mode, you can revert to the 4-Bit mode with PCB to benefit from increased data transfer rates. Using the previous configuration, switch back to `4 Bit`.


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

After flashing both the co-processor and host devices, follow these steps to connect and test your ESP-Hosted SDIO setup:

1. Connect the hardware:
   - Follow the pin assignments for SDIO as specified in [Hardware Setup](#4-hardware-setup).
   - Ensure all necessary connections are made, including power, ground, and the extra GPIO signals (Data_Ready and Reset).

2. Power on both devices. Apply correct input rating power for both chipsets.

3. Verify the connection:
   - Check the serial output of both devices for successful initialization messages.
   - Look for messages indicating that the SDIO transport layer has been established.

4. Logs at both sides:
   - Host:

     ```
     I (522) transport: Attempt connection with slave: retry[0]
     I (525) transport: Reset slave using GPIO[54]
     I (530) os_wrapper_esp: GPIO [54] configured
     I (535) gpio: GPIO[54]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
     I (1712) transport: Received INIT event from ESP32 peripheral
     I (1712) transport: EVENT: 12
     I (1712) transport: EVENT: 11
     I (1715) transport: capabilities: 0xe8
     I (1719) transport: Features supported are:
     I (1724) transport:        - HCI over SDIO
     I (1728) transport:        - BLE only
     I (1732) transport: EVENT: 13
     I (1736) transport: ESP board type is : 13

     I (1741) transport: Base transport is set-up
     ```

   - Co-processor:

     ```
     I (492) fg_mcu_slave: *********************************************************************
     I (501) fg_mcu_slave:                 ESP-Hosted-MCU Slave FW version :: X.Y.Z

     I (511) fg_mcu_slave:                 Transport used :: SDIO
     I (520) fg_mcu_slave: *********************************************************************
     I (529) fg_mcu_slave: Supported features are:
     I (534) fg_mcu_slave: - WLAN over SDIO
     I (538) h_bt: - BT/BLE
     I (541) h_bt:    - HCI Over SDIO
     I (545) h_bt:    - BLE only
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
   - Consider using a lower clock speed or checking your [hardware setup](docs/sdio.md#7-hardware-setup) if you experience communication problems.
   - ESP-Hosted-MCU troubleshooting guide: [docs/troubleshooting.md](docs/troubleshooting.md)

9. Monitoring and debugging:
   - Use the serial monitor on both devices to observe the communication between the host and co-processor.
   - For more detailed debugging, consider using a logic analyzer to examine the SDIO signals.
   - Use a logic analyzer or oscilloscope to verify the SDIO signals.
   - Ensure that the power supply to both devices is stable and within the required voltage levels.

## 9 References

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 Hardware Design Guidelines](https://www.espressif.com/en/products/hardware/esp32/resources)
- [SDIO Protocol Basics](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface)
- [ESP SDIO Slave Communication](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/protocols/esp_sdio_slave_protocol.html)

