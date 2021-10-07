## ESP-Hosted OTA feature

Currently, new OTA image supported size is upto 1MB.

There are basic three control path commands for OTA are provided in C and python, as follows:
* esp_ota_begin() -- Sets available OTA partition in flash for OTA write operation and erase it.
* esp_ota_write() -- Write chunk of OTA image data on OTA partition in flash.
* esp_ota_end()   -- Validate written OTA image, set OTA partition for next boot and reboot ESP32 after 5 sec.

Defination of these commands are present in [commands.c](host/host_common/commands.c) and [commands.py](host/linux/host_control/python_support/commands.py) files.

## How to use

### On ESP32 side
Build and flash ESP-Hosted application using `idf.py build flash monitor`.

### On Host side

#### Note
1. Please stop ongoing BT/BLE operations before starting OTA process.
2. OTA operation should not get interrupted by any other control path command.

Perform station connect using [station_connect.py](host/linux/host_control/python_support/station_connect.py) script or [test.c](host/linux/host_control/c_support/test.c), which ups `ethsta0` interface. Assign IP using dhclient.

#### OTA Using Python3
* ota_update.py

[ota_update.py](host/linux/host_control/python_support/ota_update.py) python script is used to do OTA update on ESP32. It downloads chunk of OTA image data using HTTP client over `ethsta0` interface and writes on ESP32. After successful completion it restarts ESP32 after 5 sec.

Usage:
1. start HTTP server on a remote machine which contains OTA image.
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

#### OTA Using C

[test.c](host/linux/host_control/c_support/test.c) is a test application to test control path command. `./test.out ota "ota_image"` writes chunk of already downloaded OTA image data on ESP32. After successful completion it restarts ESP32 after 5 sec.

#### Note
The OTA update using C currently assumes the complete binary is available.
User can do OTA update similar to python method using third party HTTP library.

Usage:
1. Run `make` in `host/linux/host_control/c_support` directory.
2. Download OTA image binary on host machine.
3. Pass OTA image binary path as next comamnd line argument after `ota`.

Example:
```
./test.out ota "/path/to/ota_image.bin"
```
