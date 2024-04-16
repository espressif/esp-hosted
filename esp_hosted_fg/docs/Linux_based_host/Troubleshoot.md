# Troubleshoot Instructions
## 1 SDIO
### 1.1 Host fails to detect SDIO ESP peripheral
1. Make sure to use ESP32 wrover kit. If you are using a different ESP32 module, please check [SDIO pull up requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)
2. Recheck jumper cable connections. Try to use cables that are smaller in length(less than 10 cm should work).
3. Make sure that driver module is loaded.
```
$ sudo lsmod | grep esp32
esp32                  28672  0
```
4. Check if host could perform sdio level enumeration. Assuming ESP32 gets detected as mmc1, execute following and check the output.
```
$ sudo cat /sys/devices/platform/soc/fe300000.mmc/mmc_host/mmc1/mmc1\:0001/mmc1\:0001\:1/uevent
SDIO_CLASS=00
SDIO_ID=6666:2222
MODALIAS=sdio:c00v6666d2222
```
5. Possible that SDIO timing can be wrong, because of which you might face some error messages in `/var/log/kern.log`
Add [.timing = SDIO_SLAVE_TIMING_NSEND_PSAMPLE](https://github.com/espressif/esp-idf/blob/454aeb3a48ac2b92cfa9d8b6a01d1b53179ec50a/components/hal/include/hal/sdio_slave_types.h#L26-L38) of sdio_slave_config_t in `esp/esp_driver/network_adapter/main/sdio_slave_api.c`
6. In case issue persists, collect and send following logs to Espressif support.
* dmesg or /var/log/kern.log log on host
* Output of above mentioned commands
* ESP console log

### 1.2 Unknown symbol error while executing rpi_init.sh
If user gets below dmesg or /var/log/kern.log log
```
[11827.359298] esp32_sdio: Unknown symbol sdio_release_host (err 0)
[11827.359308] esp32_sdio: Unknown symbol sdio_disable_func (err 0)
[11827.359322] esp32_sdio: Unknown symbol sdio_claim_host (err 0)
[11827.359326] esp32_sdio: Unknown symbol sdio_memcpy_fromio (err 0)
[11827.359337] esp32_sdio: Unknown symbol sdio_register_driver (err 0)
[11827.359357] esp32_sdio: Unknown symbol sdio_memcpy_toio (err 0)
[11827.359368] esp32_sdio: Unknown symbol sdio_release_irq (err 0)
[11827.359373] esp32_sdio: Unknown symbol sdio_unregister_driver (err 0)
[11827.359402] esp32_sdio: Unknown symbol sdio_claim_irq (err 0)
[11827.359406] esp32_sdio: Unknown symbol sdio_enable_func (err 0)
[11827.359417] esp32_sdio: Unknown symbol sdio_readb (err 0)
[11827.359421] esp32_sdio: Unknown symbol sdio_writeb (err 0)
```
It indicates sdhci is not compiled as a part of kernel.
Run below command before execution of rpi_init.sh
```
sudo modprobe sdhci
```

### 1.3 Flashing error, 'MD5 of file does not match data in flash'
:warning: This issue is only applicable to ESP32 SDIO
if user experiences issues while flashing the ESP32 chipset on SDIO, similar to next logs:
```
esptool.py v4.7.dev3
Serial port /dev/cu.usbserial-1301
Connecting......
Chip is ESP32-D0WD-V3 (revision v3.0)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: e0:e2:e6:26:e6:bc
Uploading stub...
Running stub...
Stub running...
Configuring flash size...
Flash will be erased from 0x00001000 to 0x00007fff...
Flash will be erased from 0x00010000 to 0x0009cfff...
Flash will be erased from 0x00008000 to 0x00008fff...
Compressed 28384 bytes to 17743...
Writing at 0x00001000... (50 %)
Writing at 0x0000781b... (100 %)
Wrote 28384 bytes (17743 compressed) at 0x00001000 in 1.6 seconds (effective 139.1 kbit/s)...
File  md5: 0efca5d0d8ef18d8756e9eed704e3ec8
Flash md5: eaa29e553c3a113188748ed1b9ea5778
MD5 of 0xFF is fe81cfe59e598e57b67baa9d08691595

A fatal error occurred: MD5 of file does not match data in flash!
CMake Error at run_serial_tool.cmake:66 (message):

  /Users/yogesh/.espressif/python_env/idf5.1_py3.11_env/bin/python;;/Users/yogesh/code/idf3/components/esptool_py/esptool/esptool.py;--chip;esp32
  failed.
```
Issue could be due to DAT2 being strapping pin conflicts with SDIO.
Please refer to https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/api-reference/peripherals/sd_pullup_requirements.html#conflicts-between-bootstrap-and-sdio-on-dat2
where user can irreversibly burn eFuses, to set the DAT2 voltage always to 3.3V.

## 2 Wi-Fi / Network
### 2.1 Network interfaces are not seen on host
Network interfaces are by default in down state. Execute `ifconfig -a` to see those.
In case issue persists, collect and send following logs to Espressif support.
* dmesg or /var/log/kern.log log on host
* Output of above mentioned commands
* ESP console log

### 2.2 WLAN datapath does not work
1. Check ESP console log for wlan disconnect event. For reconnection, execute provided python script.
2. Execute `route -n` command on host and verify that appropriate routes are configured.
3. In case issue persists, collect and send following logs to Espressif support.
* dmesg or /var/log/kern.log log on host
* Output of above mentioned commands
* ESP console log
* WLAN air capture log

## 3 Bluetooth
### 3.1 Bluetooth does not work
1. Make sure that bluetooth is not blocked on host
```
$ sudo rfkill list
1: hci0: Bluetooth
    Soft blocked: no
    Hard blocked: no
```
In case soft blocked,
```sh
$ sudo rfkill unblock bluetooth
$ sudo rfkill list # should be not blocked now
```
2. Execute `hciconfig` command to ensure that device is detected and initialized properly
3. User permissions
```sh
$ sudo usermod -G bluetooth -a $(whoami)
```
This would add current user to bluetooth group. you can change $(whoami) to username if needed, desired to to add bluetooth permission

4. Reinstall bluetooth software
```sh
$ sudo apt remove bluez bluez-firmware pi-bluetooth
$ sudo apt install bluez bluez-firmware pi-bluetooth
```

5. Restart bluetooth service
```sh
$ sudo systemctl restart bluetooth
$ sudo systemctl status bluetooth
```

6. In case issue persists, collect and send following logs to Espressif support.
* dmesg or /var/log/kern.log log on host
* Output of above mentioned commands
* ESP console log
* hcidump log (`hcidump -X -t`)

### 3.2 In case of Bluetooth over UART getting timeouts for tx
If prints like
```sh
I (17761) hci_uart: uart rx break
I (17761) hci_uart: uart rx break
I (17761) hci_uart: uart rx break
I (17761) hci_uart: uart rx break
I (17771) hci_uart: uart rx break
I (17771) hci_uart: uart rx break
I (17771) hci_uart: uart rx break
I (23761) hci_uart: uart rx break
```
coming continuously, Please verify your 'uart' baud_rate is correct and restart hciattach.
```sh
$ sudo killall hciattach
$ sudo hciattach -s <baud_rate> /dev/serial0 any <baud_rate> flow
```

## 4. SPI

### 4.1 SPI Clock is lower than expected on Raspberry Pi

This issue has been reported for [Raspberry Pi 3](https://github.com/raspberrypi/linux/issues/2286) and [Raspberry Pi 4](https://github.com/raspberrypi/linux/issues/3381).

For Raspberry Pi OS prior to _Bookworm_, edit the _/boot/config.txt_ file and add these lines:
```
core_freq=250
core_freq_min=250
```

For _Bookworm_, see [Setting the correct SPI Clock on Raspberry-Pi](SPI_setup.md#121-setting-the-correct-spi-clock-on-raspberry-pi).

### 4.2 Reset Pin and SPI GPIO numbering

On older Raspberry Pi OS (before March 2024), the GPIO numbers used for the `resetpin` parameter in `rpi_init.sh` and assigned to `HANDSHAKE_PIN` and `SPI_DATA_READY_PIN` in `esp_spi.h` should match the actual Raspberry Pi GPIOs on the header.

On newer Raspberry Pi OS (after March 2024), the GPIO numbers have been remapped. See the [Porting Guide](porting_guide.md#241-gpio-numbering-in-raspberry-pi-os) for more information.
