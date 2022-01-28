# C Demo Application

[test.c](../host/linux/host_control/c_support/test.c) is a demo application to provide basic command line arguments as follows:

| Command line argument | Operation |
|:----|:----|
| sta_connect | Connect ESP32 station to external AP, assign MAC address of ESP32 station to `ethsta0` and up `ethsta0` interface |
| sta_disconnect | Disconnect ESP32 station from external AP and down `ethsta0` interface |
| ap_start | Start ESP32 softAP, assign MAC address of ESP32 softAP to `ethap0` and up `ethap0` interface |
| ap_stop | Stop ESP32 softAP stop and down `ethap0` interface |
| scan | Scan external access points |
| sta_list | List external stations connected to softAP |
| ap_vendor_ie | Set vendor information element for ESP32 softAP |
| wifi_tx_power | sets WiFi maximum transmitting power and get WiFi current transmitting power |
| ota </path/to/ota_image.bin> | performs OTA operation using local OTA binary file |

It uses APIs present in [test_api.c](../host/linux/host_control/c_support/test_api.c). User should first modify configuration parameters in [test_config.h](../host/linux/host_control/c_support/test_config.h). Then run `make` in [c_support](../host/linux/host_control/c_support) to compile `test.c`.

Note:-
Please execute `test.out` as below.

```
ex.

Usage: sudo ./test.out [scan] [sta_connect] [sta_disconnect] [ap_start] [ap_vendor_ie] [sta_list] [ap_stop] [wifi_tx_power] [ota <Esp_binary_path>]

For example: sudo ./test.out scan sta_connect sta_disconnect ap_start ap_vendor_ie sta_list ap_stop wifi_tx_power ota <Esp_binary_path>

```
Note:
* After `sta_connect`, User needs to run DHCP client to obtain IP address from an external AP. Then network data path will be open for higher applications to use `ethsta0` interface for data communication. For an example as below.

```
sudo dhclient ethsta0 -r

sudo dhclient ethsta0 -v
```

* `ap_vendor_ie` needs to get called before starting of ESP32 softAP, please edit function `test_set_vendor_specific_ie` in `test_api.c`.
`ap_vendor_ie` should get configured only once till ESP32 reboot. To remove
previous configuration set `enable` flag to `false` in `wifi_set_vendor_specific_ie` API.
After that re-configuration possible of Vendor IE.

* After `ap_start` to start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`). For an example as below:

```
sudo dnsmasq --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h

sudo ifconfig ethap0 192.168.4.5
```

* OTA -
The OTA update using C currently assumes the complete binary is downloaded locally.
OTA update using HTTP URL is only supported in [python](python_demo.md#ota-update). In case HTTP based OTA update is desired, user can do the same using third party HTTP client library.

```
ex.
./test.out ota </path/to/ota_image.bin>
```

# C stress Application

[stress.c](../host/linux/host_control/c_support/stress.c) use for stress testing of control path APIs. It provides basic command line arguments as follows:

| Command line argument | Operation |
|:----|:----|
| Number of test iterations | Number of iterations for stress test |
| sta_connect | Connect ESP32 station to external AP, assign MAC address of ESP32 station to `ethsta0` and up `ethsta0` interface |
| sta_disconnect | Disconnect ESP32 station from external AP and down `ethsta0` interface |
| ap_start | Start ESP32 softAP, assign MAC address of ESP32 softAP to `ethap0` and up `ethap0` interface |
| ap_stop | Stop ESP32 softAP stop and down `ethap0` interface |
| scan | Scan external access points |
| sta_list | List external stations connected to softAP |
| ap_vendor_ie | Set vendor information element for ESP32 softAP |
| wifi_tx_power | sets WiFi maximum transmitting power and get WiFi current transmitting power |

Run `make stress` in [c_support](../host/linux/host_control/c_support) directory to compile `stress.c`.

Note:-
Please execute `stress.out` as below.

```
ex.
usage: sudo ./stress.out <Number of test iterations> [scan] [sta_connect] [sta_disconnect] [ap_start] [sta_list] [ap_stop] [wifi_tx_power]

For example: sudo ./stress.out 100 scan sta_connect sta_disconnect ap_start sta_list ap_stop wifi_tx_power

```
