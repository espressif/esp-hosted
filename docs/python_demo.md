# Demo Application

[test.py](../host/linux/host_control/python_support/test.py) is a demo application to test control path interface:

It uses APIs present in [test_api.py](../host/linux/host_control/python_support/test_api.py). User should first modify configuration parameters in [test_config.py](../host/linux/host_control/python_support/test_config.py).

Note:-
Please execute `test.py` as below.

```
sudo python test.py
```
Note:
* After `test_station_mode_connect` API call, User needs to run DHCP client to obtain IP address from an external AP. Then network data path will be open for higher applications to use `ethsta0` interface for data communication. For an example as below.

```
sudo dhclient ethsta0 -r

sudo dhclient ethsta0 -v
```

* After `test_softap_mode_start` API ,to start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`). For an example as below:

```
sudo dnsmasq --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h

sudo ifconfig ethap0 192.168.4.5
```

# Stress Application

[stress.py](../host/linux/host_control/python_support/stress.py) use for stress testing of control path APIs. User should first modify configuration parameters in [test_config.py](../host/linux/host_control/python_support/test_config.py). `STRESS_TEST_COUNT` variable is defined in `stress.py` for number of iterations for stress testing.

Note:-
Please execute `stress.py` as below.
```
ex.
sudo python stress.py
```

# OTA update

* `ota_update.py` is script which placed in `host/linux/host_control/python_support/` directory. Use below command to navigate to this directory.
```sh
$ cd host/linux/host_control/python_support/
```

* This script assumes station is connected to AP, IP is assigned to `ethsta0` and HTTP URL is accessible.

* [ota_update.py](host/linux/host_control/python_support/ota_update.py) python script is used to do OTA update on ESP32. It downloads **chunk** of OTA image data using HTTP client over `ethsta0` interface and writes on ESP32. After successful completion it restarts ESP32 after 5 sec.

Usage:
1. Start HTTP server on a remote machine which contains OTA image.
Following python command can be used to start HTTP server if it's not running already.

Syntax:
```
python3 -m http.server <port_number>
```
Example:
```
python3 -m http.server 9999
```

2. Pass OTA image URL as command line argument to ota_update.py.

Syntax:
```
python3 ota_update.py "http://<IP_address>:<port_number>/ota_image.bin"
```
Example:
```
python3 ota_update.py "http://192.168.0.106:9999/network_adapter.bin"
```

3. It will performs following operations.
* Erase ota flash partition of ESP32
* Download chunk from URL and write that chunk into flash, one by one, till whole binary is written
* Validate the complete written binary in flash
* Sets newly written OTA partition as boot partition
* Reboot the ESP32 after 5 second
