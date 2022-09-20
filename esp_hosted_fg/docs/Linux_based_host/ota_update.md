## **ESP-Hosted OTA update ESP image**

Please note, Maximum size of New OTA image binary depends upon -
1. Total flash size available
2. OTA partition size in partition table
As per current limits, upto 1MB binary size is supported which is configurable using custom partition table CSV. Please refer [partition tables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html) for more info.

- There are basic three [control path APIs](../common/ctrl_apis.md#1-control-path-apis) for OTA are provided
- C and python demo apps make use of them to do OTA update
  - [ota_begin()](../common/ctrl_apis.md#123-ctrl_cmd_t-ota_beginctrl_cmd_t-req)
    - Sets available OTA partition in flash for OTA write operation and erase it
  - [ota_write()](../common/ctrl_apis.md#124-ctrl_cmd_t-ota_writectrl_cmd_t-req)
    - Write chunk of OTA image data on OTA partition in flash
  - [ota_end()](../common/ctrl_apis.md#125-ctrl_cmd_t-ota_endctrl_cmd_t-req)
    - Validate written OTA image, set OTA partition for next boot and reboot ESP after 5 sec

## **How to use**

### On ESP side
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

2. For further details follow this [Link](../common/python_demo.md#ota-update).

### C Implementation

For Further details follow this [Link](../common/c_demo.md#some-points-to-note) -> `OTA`.
