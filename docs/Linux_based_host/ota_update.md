## **ESP-Hosted OTA update ESP32 image**

Please note, Maximum size of New OTA image binary depends upon -
1. Total flash size available
2. OTA partition size in partition table
As per current limits, upto 1MB binary size is supported which is configurable using custom partition table CSV. Please refer [partition tables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html) for more info.

There are basic three control path commands for OTA are provided in C and python, as follows:
* esp_ota_begin() -- Sets available OTA partition in flash for OTA write operation and erase it.
* esp_ota_write() -- Write chunk of OTA image data on OTA partition in flash.
* esp_ota_end()   -- Validate written OTA image, set OTA partition for next boot and reboot ESP32 after 5 sec.

Definition of these commands are present in [commands.c](host/host_common/commands.c) and [commands_lib.py](host/linux/host_control/python_support/commands_lib.py) files.

## **How to use**

### On ESP32 side
Build and flash ESP-Hosted application using `idf.py build flash monitor`.

### On Host side

#### Note
1. Please stop ongoing BT/BLE operations before starting OTA process.
2. OTA operation should not get interrupted by any other control path command.
3. OTA update using HTTP URL is only supported in python. In case HTTP based OTA update is desired, user can do the same using third party HTTP client library.
4. The OTA update using C currently assumes the complete binary is downloaded locally.

### Python Implementation
User can skip step 1, if station is connected to AP and IP is assigned to `ethsta0` interface.

1. Perform station connect using [wifi station mode operations](Getting_started.md#111-wi-fi-station-mode-operations)
-> Connect to external access point.

2. For further details follow this [Link](../python_demo.md#ota-update).

### C Implementation

For Further details follow this [Link](../c_demo.md#c-demo-application).
