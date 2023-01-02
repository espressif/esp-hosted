# Wi-Fi connectivity Setup over SDIO
## 1. Setup
### 1.1 Hardware Setup/Connections
* In this setup, ESP board acts as a SDIO peripheral and provides Wi-Fi capabilities to host. Please connect ESP peripheral to STM32F412ZGT6-Nucleo 144 board's CN8 Extension connecter with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity.
* Power ESP peripheral and STM32F412ZGT6-Nucleo 144 separately with a power supply that provide sufficient power. ESP peripheral and STM32 can be powered through USB-Hub using micro-USB cable. It is also used as USART connection for debug logs from host. Serial port communicaton program like tera term or minicom used to print the logs.
* BT/BLE support will be added in upcoming release.

#### Hardware connections for ESP32
| STM32 Pin | ESP32 Pin | Function |
|:----------:|:---------:|:--------:|
| PC8 (pin2) | IO2 | DATA0 |
| PC9 (pin4) | IO4 | DATA1 |
| PC10 (pin6) | IO12 | DATA2 |
| PC11 (pin8) | IO13 | DATA3 |
| PC12 (pin10)| IO14 | Clock |
| PD2 (pin12) | IO15 | Command |
| GND (pin13) | GND | Ground |
| PG2 (pin14) | EN | Reset ESP |

Setup image is here.

![alt text](stm32_esp32_sdio_setup.jpg "setup of STM32F412ZGT6-Nucleo 144 as host and ESP32 as peripheral")

# 2. ESP peripheral setup
## 2.1 ESP-IDF requirement
:warning:`Note: ESP-IDF is needed to compile ESP-Hosted firmware source. Skip this step if you are planning to use pre-built release binaries.`

- Clone the ESP-IDF [release/v5.0](https://github.com/espressif/esp-idf/tree/release/v5.0)  and checkout to `release/v5.0` branch.
- The control path between MCU host and ESP peripheral is based on `protobuf`. For that, corresponding stack layer, `protocomm` from ESP-IDF is used. It will be already present in ESP-IDF, no extra setup required for that.

### 2.2 Setup
#### 2.2.1 Using pre-built binary
* Download pre-built firmware binaries from [releases](https://github.com/espressif/esp-hosted/releases)
* Follow `readme.txt` from release tarball to flash the ESP binary
* :warning: Make sure that you use `Source code (zip)` in `Assets` fold with associated release for host building.
* Windows user can use ESP Flash Programming Tool to flash the pre-built binary.

#### 2.2.2 Compilation using source

- Note: Please use the same git commit both at ESP and Host
- Clone the ESP-IDF [release/v5.0](https://github.com/espressif/esp-idf/tree/release/v5.0) and git checkout to `release/v5.0` branch.
- [Set-up the ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/release-v5.0/esp32/get-started/index.html)
- Navigate to `esp_hosted_fg/esp/esp_driver/network_adapter` directory.

##### Using cmake

```
$ idf.py fullclean
```

Run following command and navigate to `Example Configuration -> Transport layer -> SDIO interface -> select` and exit from menuconfig. Read more about [idf.py](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/build-system.html#using-the-build-system) here.
```
$ idf.py menuconfig
```

To build and flash the app on ESP peripheral, run

```sh
$ idf.py -p <serial_port> build flash
```
