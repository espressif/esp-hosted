# Demo App in C

### Introduction
- This demo application showcases a way to configure control path.
- App is avaliable under directory [c_support/](../../host/linux/host_control/c_support)
- App interacts with `Hosted Control Library` using [control path APIs](./ctrl_apis.md). APIs header could be found at [ctrl_api.h](../../host/control_lib/include/ctrl_api.h)

### Source files
- This demo app works over [Hosted Control Library](../../host/control_lib/)
- Source files are [test.c](../../host/linux/host_control/c_support/test.c) and [test_utils.c](../../host/linux/host_control/c_support/test_utils.c)
- Config file : [ctrl_config.h](../../host/linux/host_control/c_support/ctrl_config.h)

### List of control path commands
- Below is a list of control path commands showcased in demo app

| Command line argument | Operation |
|:----|:----|
| get_sta_mac_addr | Get Mac address of WiFi in station mode |
| get_softap_mac_addr | Get Mac address of WiFi in softap mode |
| set_sta_mac_addr | Set Mac address of WiFi in station mode |
| set_softap_mac_addr | Set Mac address of WiFi in softap mode |
|||
| get_wifi_mode | Get current WiFi mode |
| set_wifi_mode | Set WiFi mode |
|||
| get_ap_scan_list | Get neighboring AP (WiFi routers or Hotspot) list |
| sta_connect | Connect ESP32 station to external AP(WiFi router or hotspot), assign MAC address of ESP32 station to `ethsta0` and up `ethsta0` interface |
| get_sta_config | Get details of connected AP in station mode |
| sta_disconnect | Disconnect ESP32 station from external AP and down `ethsta0` interface |
|||
| set_softap_vendor_ie | Set vendor information element for ESP32 softAP |
| reset_softap_vendor_ie | Reset vendor information element for ESP32 softAP |
| softap_start | Start ESP32 softAP, assign MAC address of ESP32 softAP to `ethap0` and up `ethap0` interface |
| get_softap_config | Get softAP configuration |
| softap_connected_sta_list | Get connected station info of softap |
| softap_stop | Stop ESP32 softAP stop and down `ethap0` interface |
|||
| set_wifi_powersave_mode | Set WiFi Power save mode |
| get_wifi_powersave_mode | Get WiFi Power save mode |
|||
| set_wifi_max_tx_power | Sets WiFi maximum transmitting power |
| get_wifi_curr_tx_power | Get WiFi current transmitting power |
|||
| ota </path/to/ota_image.bin> | performs OTA operation using local OTA binary file |


### How to run
It uses APIs present in [ctrl_api.h](../../host/control_lib/include/ctrl_api.h). User should first modify configuration parameters in [ctrl_config.h](../../host/linux/host_control/c_support/ctrl_config.h). Then run `make` in [c_support/](../../host/linux/host_control/c_support) to compile `test.c`.

Please execute `test.out` as below.

```sh
$ make
$ sudo ./test.out \
	[ get_sta_mac_addr      || get_softap_mac_addr     || set_sta_mac_addr          || \
	  set_softap_mac_addr   || get_wifi_mode           || set_wifi_mode             || \
	  get_ap_scan_list      || sta_connect             || get_sta_config            || \
	  sta_disconnect        || set_softap_vendor_ie    || reset_softap_vendor_ie    || \
	  softap_start          || get_softap_config       || softap_connected_sta_list || \
	  softap_stop           || set_wifi_powersave_mode || get_wifi_powersave_mode   || \
	  set_wifi_max_tx_power || get_wifi_curr_tx_power  || \
	  ota </path/to/esp_firmware_network_adapter.bin> \
	]
```
For example,
```sh
$ sudo ./test.out get_wifi_mode
```

### Some points to note
- Connect to AP in station mode
  - After `sta_connect`, User needs to run DHCP client to obtain IP address from an external AP. Then network data path will be open for higher applications to use `ethsta0` interface for data communication. For an example as below.

  ```sh
  $ sudo dhclient ethsta0 -r

  $ sudo dhclient ethsta0 -v
  ```

- Disconnect AP in station mode
  - After disconnect, user can remove DHCP lease. For example,

  ```sh
  $ sudo dhclient ethsta0 -r
  ```

- For softAP vendor specific IE
  - `set_softap_vendor_ie` will be in effect only if it is done before starting of ESP32 softAP
  - Once vendor IE set, consecutive `set_softap_vendor_ie` will fail unless vendor IE is reset using `reset_softap_vendor_ie` or ESP32 reboot
- SoftAP start
  - After `softap_start` to start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`). For an example as below:

  ```sh
  $ sudo dnsmasq --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h

  $ sudo ifconfig ethap0 192.168.4.5
  ```

- SoftAP stop
  - After stoppong softAP, dnsmasq could be down/killed as per user expectation
- OTA
The OTA update using C currently assumes the complete binary is downloaded locally.
OTA update using HTTP URL is only supported in [python](python_demo.md#ota-update). In case HTTP based OTA update is desired, user can do the same using third party HTTP client library.

  ```sh
  ex.
  ./test.out ota </path/to/ota_image.bin>
  ```

# C stress Application

[stress.c](../../host/linux/host_control/c_support/stress.c) use for stress testing of control path APIs. It is very similar to demo app, just additionally allows multiple iteration testing

### Source files
- Source files are [stress.c](../../host/linux/host_control/c_support/stress.c) and [test_utils.c](../../host/linux/host_control/c_support/test_utils.c)
- Config file : [ctrl_config.h](../../host/linux/host_control/c_support/ctrl_config.h)

### How to run
- Change expected configuration from config file
- Run `make stress` in [c_support](../../host/linux/host_control/c_support) directory to compile `stress.c`.
- Please execute `stress.out` as below.

```sh
$ sudo ./stress.out <Number of test iterations> [get_sta_mac_addr] [get_softap_mac_addr] \
		[set_sta_mac_addr] [set_softap_mac_addr] [get_wifi_mode] [set_wifi_mode] \
		[get_ap_scan_list] [sta_connect] [get_sta_config] [sta_disconnect] \
		[set_softap_vendor_ie] [reset_softap_vendor_ie] [softap_start] \
		[get_softap_config] [softap_connected_sta_list] [softap_stop] \
		[set_wifi_powersave_mode] [get_wifi_powersave_mode] [set_wifi_max_tx_power] \
		[get_wifi_curr_tx_power] [ota </path/to/esp_firmware_network_adaptor.bin>]

For example:
$ sudo ./stress.out 10 scan sta_connect sta_disconnect ap_start sta_list ap_stop wifi_tx_power

```
