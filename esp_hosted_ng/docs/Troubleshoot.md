# Troubleshoot Instructions
## 1. Host fails to detect SDIO ESP peripheral
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

## 2. Network interfaces are not seen on host
Network interfaces are by default in down state. Execute `ifconfig -a` to see those.
In case issue persists, collect and send following logs to Espressif support.
* dmesg log on host
* Output of above mentioned commands
* ESP console log

## 3. WLAN datapath does not work
1. Check ESP console log for wlan disconnect event. For reconnection, execute provided python script.
2. Execute `route -n` command on host and verify that appropriate routes are configured.
3. In case issue persists, collect and send following logs to Espressif support.
* dmesg log on host
* Output of above mentioned commands
* ESP console log
* WLAN air capture log

## 4. Bluetooth does not work
1. Make sure that bluetooth is not blocked on host
```
$ sudo rfkill list
1: hci0: Bluetooth
    Soft blocked: no
    Hard blocked: no
```
2. Execute `hciconfig` command to ensure that device is detected and initialized properly
3. User permissions
```sh
$ sudo usermod -G bluetooth -a pi
```

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
* dmesg log on host
* Output of above mentioned commands
* ESP console log
* hcidump log (`hcidump -X -t`)

## 5. Unknown symbol error while executing rpi_init.sh
If user gets below dmesg log
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
