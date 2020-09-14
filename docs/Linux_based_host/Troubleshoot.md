# Troubleshoot instructions
## Host fails to detect esp device
1. Make sure to use ESP32 wrover kit. If you are using a different ESP32 module, please check pull up requirements (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)
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
5. In case issue persists, collect and send following logs to Espressif support.
* dmesg log on host
* Output of above mentioned commands
* ESP32 console log

## Network interfaces are not seen on host
Network interfaces are by default in down state. Execute `ifconfig -a` to see those.
In case issue persists, collect and send following logs to Espressif support.
* dmesg log on host
* Output of above mentioned commands
* ESP32 console log

## WLAN datapath does not work
1. Check ESP32 console log for wlan disconnect event. For reconnection, execute provided python script.
2. Execute `route -n` command on host and verify that appropriate routes are configured.
3. In case issue persists, collect and send following logs to Espressif support.
* dmesg log on host
* Output of above mentioned commands
* ESP32 console log
* WLAN air capture log

## Bluetooth does not work
1. Make sure that bluetooth is not blocked on host
```
$ sudo rfkill list
1: hci0: Bluetooth
    Soft blocked: no
    Hard blocked: no
```
2. Execute `hciconfig` command to ensure that device is detected and initialized properly
3. In case issue persists, collect and send following logs to Espressif support.
* dmesg log on host
* Output of above mentioned commands
* ESP32 console log
* hcidump log (`hcidump -X -t`)
