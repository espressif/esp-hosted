# ESP-Hosted: Co-Processor External Coexistence (EXT_COEX)

This host side example demonstrates how to configure and use **External Coexistence (EXT_COEX)** in **ESP-Hosted** co-processor.

The ESP co-processor (slave) has a built-in hardware **Packet Traffic Arbitrator (PTA)** that manages 2.4 GHz ISM band access between its own Wi-Fi and an external peer radio (such as BLE, Zigbee, or Thread running on the host MCU). In a typical standalone setup, this PTA is configured directly on the ESP device. With ESP-Hosted, the host MCU can configure and control the slave's PTA remotely over RPC — without modifying slave firmware.

The arbitration operates over 1, 2, or 3 GPIO lines depending on the chosen mode:

- **1-wire** — The peer radio asserts **Request** whenever it needs the band; ESP Wi-Fi always yields. Simple, but can impact Wi-Fi performance.
- **2-wire** — Adds a **Grant** signal. The PTA arbitrates and returns the result: Grant high means the peer radio may transmit; Grant low means ESP Wi-Fi may transmit.
- **3-wire** — Adds a **Priority** signal, allowing the peer radio to indicate middle or high priority. The PTA compares this against the internal Wi-Fi priority before granting.

## 1 Supported Platforms

**Host MCU**

| Platform |
| -------- |
| ESP32 |
| ESP32-C Series |
| ESP32-S Series |

**Co-processor (Slave)**

| Platform |
| -------- |
| ESP32-C6 / C61 |
| ESP32-C5 |
| ESP32-S3 |

> **Note:** ESP32 as a co-processor is not supported. Bluetooth on the co-processor is not supported.

## 2 Setup

EXT_COEX requires configuration on both the **slave** and the **host** before use.

### 2.1 Slave Setup

Run `eh.py menuconfig` on the slave and apply the following:

```
    ├── Component config → Bluetooth
    │    └── [ ] Bluetooth                                   <---- DISABLE ✘
    └── Example Configuration
         ├── [ ] Enable BT sharing via hosted                <---- DISABLE ✘
         └── [*] External Coexistence support                <---- Enable  ✔
```

Flash the slave:

```bash
eh.py -p <cp_usb_serial_port> flash monitor
```

### 2.2 Host Setup

Run `eh.py menuconfig` on the host and enable the following:

```
Component config → ESP-Hosted config
    ├── [*] Co-Processor External Coexistence                <---- Enable ✔
    └── [*] Co-Processor External Coexistence - Advanced     <---- Enable ✔
```

Flash the host:

```bash
eh.py -p <host_usb_serial_port> flash monitor
```

## 3 Host-Side APIs

Include the following header to access all EXT_COEX APIs:

```c
#include "esp_hosted.h"
```

This pulls in `esp_hosted_cp_ext_coex.h`, which exposes the APIs below.

#### 3.1 `esp_hosted_cp_ext_coex_set_work_mode()`

```c
esp_err_t esp_hosted_cp_ext_coex_set_work_mode(
    esp_hosted_ext_coex_work_mode_t work_mode
);
```

Sets the coexistence operating mode, determining whether the host leads or follows in radio arbitration.

**Parameters**

- `work_mode` — Role to assign to the host:
  - `ESP_HOSTED_EXT_COEX_LEADER_ROLE` — Host acts as the leader.
  - `ESP_HOSTED_EXT_COEX_FOLLOWER_ROLE` — Host acts as the follower.
  - `ESP_HOSTED_EXT_COEX_UNKNOWN_ROLE` — Role is unspecified.

**Returns** `ESP_OK` on success, error code otherwise.

#### 3.2 `esp_hosted_cp_ext_coex_set_gpio_pin()`

```c
esp_err_t esp_hosted_cp_ext_coex_set_gpio_pin(
    uint32_t wire_type,
    const esp_hosted_ext_coex_gpio_set_t *gpio_pins
);
```

Configures the GPIO pins used for external coexistence signaling on the co-processor.

**Parameters**

- `wire_type` — Number of wires used for coexistence signaling:
  - `ESP_HOSTED_EXT_COEX_WIRE_1`
  - `ESP_HOSTED_EXT_COEX_WIRE_2`
  - `ESP_HOSTED_EXT_COEX_WIRE_3`
  - `ESP_HOSTED_EXT_COEX_WIRE_4`

- `gpio_pins` — Pointer to an `esp_hosted_ext_coex_gpio_set_t` structure with the following fields:

  | Field      | Description                          |
  | ---------- | ------------------------------------ |
  | `request`  | GPIO number for the Request signal   |
  | `priority` | GPIO number for the Priority signal  |
  | `grant`    | GPIO number for the Grant signal     |
  | `tx_line`  | GPIO number for the TX Line signal   |

**Returns** `ESP_OK` on success, error code otherwise.

#### 3.3 `esp_hosted_cp_ext_coex_set_grant_delay()` *(advanced)*

```c
esp_err_t esp_hosted_cp_ext_coex_set_grant_delay(uint8_t delay_us);
```

Sets the delay (in microseconds) applied to the GRANT signal for timing optimization. Available when `CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX_ADVANCE` is enabled.

**Parameters**

- `delay_us` — Grant signal delay in microseconds.

**Returns** `ESP_OK` on success, error code otherwise.

#### 3.4 `esp_hosted_cp_ext_coex_set_validate_high()` *(advanced)*

```c
esp_err_t esp_hosted_cp_ext_coex_set_validate_high(bool is_high_valid);
```

#### 3.5 `esp_hosted_cp_ext_coex_disable()`

```c
esp_err_t esp_hosted_cp_ext_coex_disable(void);
```

Disables external coexistence on the co-processor and releases the associated GPIOs.

**Returns** `ESP_OK` on success, error code otherwise.

Configures the polarity of the validate signal. Available when `CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX_ADVANCE` is enabled.

**Parameters**

- `is_high_valid`:
  - `true` — Validate signal is active-high.
  - `false` — Validate signal is active-low.

**Returns** `ESP_OK` on success, error code otherwise.

## 4 Host ↔ Slave API Mapping

| Host API | Slave API | Purpose |
| :------- | :-------- | :------ |
| `esp_hosted_cp_ext_coex_set_work_mode()` | `esp_external_coex_set_work_mode` | Set coexistence operating mode |
| `esp_hosted_cp_ext_coex_set_gpio_pin()` | `esp_enable_extern_coex_gpio_pin` | Configure coexistence GPIOs (REQ, PRI, GRANT, TX) |
| `esp_hosted_cp_ext_coex_set_grant_delay()` | `esp_external_coex_set_grant_delay` | Adjust GRANT signal delay for timing |
| `esp_hosted_cp_ext_coex_set_validate_high()` | `esp_external_coex_set_validate_high` | Set validate signal polarity |
| `esp_hosted_cp_ext_coex_disable()` | `esp_disable_extern_coex_gpio_pin` | Disable coexistence and release GPIOs |

## 5 References

- [External Coexistence Design (Application Note)](https://documentation.espressif.com/external_coexistence_design_en.pdf)
