This migration guide documents key changes in ESP-Hosted that users must be aware of when migrating from older versions.

#### Index
1. [2.5.2 - Bluetooth Controller on Co-Processor Disabled by Default](#coloryellow-text252---bluetooth-controller-on-co-processor-disabled-by-default)
2. [2.6.0 - ESP-Hosted Slave OTA](#coloryellow-text260---esp-hosted-slave-ota)
3. [2.11.0 - ESP-Hosted Host Driver](#coloryellow-text2110---esp-hosted-host-driver)
4. [2.12.4 - Custom Msg Callback - User Ptr](#coloryellow-text2124---custom-msg-callback---user-ptr)
5. [3.0.0 - Migration to the Unified Release](#coloryellow-text300---migration-to-the-unified-release)

# $${\color{yellow} \text{3.0.0 - Migration to the Unified Release}}$$

ESP-Hosted 3.x unifies MCU and Linux hosts on one RPC-V2 protocol and one
programming model. In all cases: **upgrade the co-processor first, then the
host**, and finish with both on 3.x.

Pick your host and follow only that section:

```text
          Which host are you migrating?
                      |
        +-------------+-------------+
        |                           |
    MCU host                    Linux host
 ESP32 running your          Linux CPU + kernel
 firmware (examples          module + user-space
 mcu_host / esp_host)        library
        |                           |
        v                           v
  see section A               see section B
```

----

## A. MCU host

### Upgrade order

```text
Existing deployment
   |
   v
Upgrade the 3.x co-processor firmware   (host OTAs it over the existing link)
   |
   v   co-processor reboots into 3.x
   |
   v
OTA your host application to 3.x
   |
   v
Host 3.x  +  co-processor 3.x
```

The OTA calls (`esp_hosted_slave_ota_begin OR eh_host_cp_ota_begin()` -> `write()` -> `end()` -> `activate()`) (check your existing ota method)

### Compatibility

| MCU host | Co-processor firmware | Status | Action |
| --- | --- | --- | --- |
| 3.x | 3.x | Supported | None |
| 3.x | 0.0.6+ (streaming) / 1.x / 2.x | Back-compatible | Upgrade the co-processor (recommended) |
| < 3.x | 3.x | Unsupported | Upgrade the host first |

### Bluetooth

Hosted Bluetooth moved out of the core. 2.x folded the stack-specific HCI
adapter into the hosted core; 3.x keeps the core a stack-agnostic HCI byte pipe
and moves the adapter into a per-stack bridge you add on top. The controller
stays on the co-processor and your GAP/GATT code is unchanged.

Boxes drawn with `#` are the ones that changed.

```text
2.x — HCI adapter lives inside the core

  +----------------------+
  |     Application      |
  +----------------------+
             |
  +----------------------+
  |  Bluedroid / NimBLE  |
  +----------------------+
             |
  ########################
  #    ESP-Hosted core   #   <-- HCI adapter is in here
  #  (HCI adapter here)  #
  ########################
             |
  +----------------------+
  |  BT controller (CP)  |
  +----------------------+


3.x — adapter split out into its own bridge

  +----------------------+
  |     Application      |
  +----------------------+
             |
  +----------------------+
  |  Bluedroid / NimBLE  |
  +----------------------+
             |
  ##################################
  #            HCI bridge           #   <== NEW
  #  esp_hosted_hci_bluedroid/_nim  #
  ##################################
             |
  ##################################
  #          ESP-Hosted core        #   <== CHANGED: stack-agnostic HCI byte pipe
  ##################################
             |
  +----------------------+
  |  BT controller (CP)  |
  +----------------------+
```

sdkconfig:

```diff
- CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID       # or ..._ENABLE_BT_NIMBLE
- CONFIG_ESP_HOSTED_BLUEDROID_HCI_VHCI        # or ..._NIMBLE_HCI_VHCI
+ CONFIG_ESP_HOSTED_HOST_FEAT_BT
+ # add one bridge component: esp_hosted_hci_bluedroid  OR  esp_hosted_hci_nimble
```

app_main():

```diff
+ esp_hosted_hci_<stack>_setup();   // connect + controller + attach HCI
  esp_bluedroid_init();             // or nimble_port_init() for NimBLE
  esp_bluedroid_enable();
```

Bluedroid attaches at runtime (`esp_bluedroid_attach_hci_driver()`); NimBLE via
link-time transport symbols. `esp_hosted_bt_controller_init/_enable/_disable/
_deinit()` stay source-compatible.

> [!WARNING]
> The legacy BT Kconfig (`ESP_HOSTED_ENABLE_BT_BLUEDROID`, `BLUEDROID_HCI_VHCI`,
> and the NimBLE equivalents) is removed. If a stale `sdkconfig` still sets one,
> the build fails on purpose — switch to `CONFIG_ESP_HOSTED_HOST_FEAT_BT` and add
> the HCI bridge component.

----

## B. Linux host

Linux moves from RPC-V1 (protobuf `CtrlMsg`, ESP-Hosted-FG) to RPC-V2 — the same
RPC the MCU host uses. Rebuild the kernel module and user-space library for 3.x.

### Upgrade order

```text
Existing deployment
   |
   v
Upgrade the 3.x co-processor firmware
   |
   v   co-processor reboots into 3.x
   |
   v
Rebuild kernel module + user-space library for 3.x
   |
   v
Host 3.x  +  co-processor 3.x
```

### Compatibility

A 3.x Linux host is RPC-V2 only: it does **not** back-compat FG (1.x/2.x)
slaves. FG interop runs the other way — an FG Linux host paired with a 3.x
co-processor built for that FG generation.

| Linux host | Co-processor firmware | Status |
| --- | --- | --- |
| 3.x | 3.x | Supported |
| ESP-Hosted-FG 1.x | 3.x built `CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V1` | Back-compatible; upgrade recommended |
| ESP-Hosted-FG 2.x | 3.x built `CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V2` | Back-compatible; upgrade recommended |
| ESP-Hosted-FG 1.x / 2.x | Standard 3.x co-processor | Unsupported (RPC differs) |

### Staged migration — keep an existing FG host

Not upgrading the Linux host yet? Build the 3.x co-processor for its FG
generation (one image per generation; no runtime RPC switch):

```text
Existing Linux host
   |
   +-- FG 1.x  ->  build 3.x CP with CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V1=y
   |
   +-- FG 2.x  ->  build 3.x CP with CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V2=y
   |
   v
Back-compatible during migration (upgrade recommended)
```

Bluetooth on the Linux host uses the kernel's BlueZ stack, not the hosted HCI
bridges — the MCU Bluetooth steps do not apply to Linux.

----

## C. Both hosts

### Unified RPC model

Linux now uses the same RPC as MCU hosts (previously RPC-V1 on Linux, RPC-V2 on
MCU, maintained separately). One implementation, one programming model, shared
examples, feature parity. The application flow is identical on both:

```c
esp_wifi_init();
esp_wifi_start();
esp_wifi_connect();   /* drive the rest via Wi-Fi / IP events */
```

### Limitations

- ESP-Hosted-NG has not yet migrated to RPC-V2.
- Existing ESP-Hosted-FG Linux applications are not source-compatible and
  require migration to the unified RPC interface.


----

# $${\color{yellow} \text{2.12.4 - Custom Msg Callback - User Ptr}}$$

## Migration needed from versions


| Firmware | Version    | Migration required |
| -------- | ---------- | ------------------ |
| Host     | < 2.12.4  | ✅                  |
| Slave    | < 2.12.4  | ✅                  |

## Reason for change

1. `esp_hosted_register_custom_callback()` now supports a user-provided pointer to be passed back on every callback invocation.
2. This allows external code to maintain per-callback context without global variables.

### Old API

```c
esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id,
    void (*callback)(uint32_t msg_id, const uint8_t *data, size_t data_len));

esp_err_t esp_hosted_send_custom_data(uint32_t msg_id, const uint8_t *data, size_t data_len);
```

### New API

```c
esp_err_t esp_hosted_register_custom_callback(uint32_t msg_id_exp,
    void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd, size_t data_len_recvd, void *local_context), // <-- Changed
    void *local_context); // <-- Extra argument

esp_err_t esp_hosted_send_custom_data(uint32_t msg_id_to_send, const uint8_t *data_to_send, size_t data_len_to_send); // no logical change
```

*Arguments:*

* `msg_id` – message ID to register
* :zap: `callback` – function pointer to handle the message (adds void* as last arg)
* :zap: `user` – user-provided pointer returned on every callback invocation

*Returns:* `ESP_OK` on success, or an error code on failure.


#  $${\color{yellow} \text{2.11.0 - ESP-Hosted Host Driver}}$$

## Migration needed from versions

| Host version | wifi-remote version |
| ------------ | ------------------- |
| < 2.11.0     | < 1.3.1             |

1. A double-free memory error can occur in some situations when ESP-Hosted Host receives network data and passes it to the netif `rx()` function (registered by netif via the wifi-remote component) for processing.

2. This error is resolved in ESP-Hosted v2.11.1. It must be used with wifi-remote v1.3.1 or greater to prevent a memory leak condition during netif initialization.

#  $${\color{yellow} \text{2.6.0 - ESP-Hosted Slave OTA}}$$


## Migration needed from versions

| Slave version | Host version |
| ------------- | ------------ |
| > 2.5.X      | > 2.5.X     |

## Reason for change

1. The existing `esp_hosted_slave_ota()` API was restrictive, supporting only HTTP-based OTA updates.
   The OTA APIs are now exposed so developers can implement their own OTA mechanisms.
2. The port layer previously contained OTA logic, which forced inclusion of the HTTP client in the host codebase even when not required.

## Changes required on host

If you are migrating from the old `esp_hosted_slave_ota()` function, update your code as follows.

### Old API (deprecated)

```c
#include "esp_hosted.h"

const char *image_url = "http://example.com/network_adapter.bin";
esp_err_t ret = esp_hosted_slave_ota(image_url);
if (ret != ESP_OK) {
    printf("OTA update failed[%d]\n", ret);
}
```

## New APIs

The co-processor OTA process is now performed using the following APIs.

### `esp_hosted_slave_ota_begin()`

```c
esp_err_t esp_hosted_slave_ota_begin(void);
```

Initializes the OTA process on the co-processor.

* Arguments: None
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Prepares the co-processor for firmware reception
  * Allocates OTA buffers
  * Sets up the OTA partition on the co-processor

### `esp_hosted_slave_ota_write()`

```c
esp_err_t esp_hosted_slave_ota_write(const void *data, size_t size);
```

Sends firmware data chunks to the co-processor.

* Arguments:

  * `data`: Pointer to firmware data chunk
  * `size`: Size of the data chunk (typically 1400–1500 bytes)
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Transmits firmware data over ESP-Hosted transport (SDIO/SPI/UART)
  * The co-processor writes data to its OTA partition
  * Can be called multiple times for large firmware images

### `esp_hosted_slave_ota_end()`

```c
esp_err_t esp_hosted_slave_ota_end(void);
```

Finalizes the OTA process.

* Arguments: None
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Validates the complete firmware image on the co-processor
  * Calculates and verifies checksums
  * Marks the new firmware as valid but not yet active

### `esp_hosted_slave_ota_activate()`

```c
esp_err_t esp_hosted_slave_ota_activate(void);
```

Activates the newly flashed firmware.

* Arguments: None
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Switches the co-processor’s boot partition to the new firmware
  * Triggers co-processor reboot with the new firmware
  * Note: After this call, the co-processor restarts with the new firmware

## How to use the new APIs

A dedicated example demonstrates the usage of the new OTA APIs:
[Slave OTA using ESP-Hosted transport](../examples/ota/coprocessor_ota/README.md)

> [!TIP]
> The example uses the new dedicated co-processor OTA APIs.
> You can reuse or customize it for your own OTA workflow.

Example methods supported:

| Method           | Description                                                                                                                                 |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Partition method | Slave firmware binary stored in Host’s partition table (`slave_fw` partition). Requires an extra host partition, but no Wi-Fi connectivity. |
| LittleFS method  | Host partition formatted as LittleFS and stores the co-processor firmware. Requires an extra host partition, but no Wi-Fi connectivity.            |
| HTTPS method     | Slave firmware binary hosted on an HTTPS server. No extra host partition needed, but requires Wi-Fi connectivity.                           |

# $${\color{yellow} \text{2.5.2 - Bluetooth Controller on Co-Processor Disabled by Default}}$$

## Migration needed from versions

| Slave version | Host version |
| ------------- | ------------ |
| > 2.5.1       | > 2.5.1      |

Before v2.5.2, the Bluetooth controller on the co-processor was initialized and enabled by default.
From v2.5.2 onwards, it starts in a disabled state.

## Reason for change

This allows users to modify the Bluetooth MAC address before the controller is initialized, as it can only be changed prior to enabling the controller.

## New APIs

### `esp_hosted_bt_controller_init()`

```c
esp_err_t esp_hosted_bt_controller_init(void);
```

Initializes the Bluetooth controller on the co-processor.

* Arguments: None
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Allocates and initializes controller resources
  * Prepares the controller for activation

### `esp_hosted_bt_controller_deinit()`

```c
esp_err_t esp_hosted_bt_controller_deinit(bool mem_release);
```

Deinitializes the Bluetooth controller on the co-processor.

* Arguments:

  * `mem_release`: If true, releases controller memory (cannot be reused)
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Stops the Bluetooth controller
  * Optionally releases memory used by the controller
  * Once released, the controller cannot be reinitialized without reboot

### `esp_hosted_bt_controller_enable()`

```c
esp_err_t esp_hosted_bt_controller_enable(void);
```

Enables the Bluetooth controller on the co-processor.

* Arguments: None
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Starts the Bluetooth controller task
  * Enables radio and HCI interfaces for Bluetooth operation

### `esp_hosted_bt_controller_disable()`

```c
esp_err_t esp_hosted_bt_controller_disable(void);
```

Disables the Bluetooth controller on the co-processor.

* Arguments: None
* Returns: `ESP_OK` on success, or an error code on failure
* What it does:

  * Gracefully stops the controller
  * Disables the Bluetooth radio
  * Must be called before deinitializing the controller

## Changes required on host

Before starting the Bluetooth stack on the host:

1. Call `esp_hosted_connect_to_slave()` to establish a connection with the co-processor.
2. (Optional) Set the Bluetooth MAC address using `esp_hosted_iface_mac_addr_set()`.
3. Initialize the Bluetooth controller using `esp_hosted_bt_controller_init()`.
4. Enable the Bluetooth controller using `esp_hosted_bt_controller_enable()`.

See [Initializing the Bluetooth Controller](features/bluetooth.md) for more details.

## How to use the new APIs

You can now start the host Bluetooth stack and use Bluetooth as usual.
All ESP-Hosted Bluetooth host examples (NimBLE and BlueDroid) have been updated accordingly.

For an example showing how to change the BT MAC address before starting the controller, refer to:
[BT Controller Example](../examples/bluetooth/README.md)
