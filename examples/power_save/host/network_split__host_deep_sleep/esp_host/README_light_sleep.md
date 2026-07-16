# Light Sleep Integration Guide

This feature integrates light sleep support on the **coprocessor** side.

This guide explains how to integrate **Light Sleep** power management for the ESP slave device when the host enters power save mode. Light sleep provides a balance between power savings and fast wake-up times.

## Overview

When the host enters power save mode, the slave can automatically enter light sleep to further reduce power consumption. This feature requires proper configuration of ESP-IDF power management components and ESP-Hosted light sleep options.

**Key Benefits:**

* **Power Savings**: Significantly reduces slave power consumption compared to active mode.
* **Fast Wake-up**: Much faster wake-up time compared to deep sleep.
* **Network Maintained**: WiFi and/or Bluetooth connection stays active during light sleep.
* **Automatic Integration**: Works seamlessly with host power save callbacks.

---

## 1. Slave Light Sleep API ([`slave_light_sleep.h`](https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/slave_light_sleep.h))

These core functions provide manual control over the power state. They are exposed to the application and can be used independently of the Host Power Save feature.

| Function | Purpose |
| --- | --- |
| `slave_light_sleep_init()` | Configures the Power Management (PM) framework and frequency scaling. **Must be called once during initialization.** |
| `slave_light_sleep_start()` | Releases the PM lock to allow the CPU to enter light sleep automatically during FreeRTOS idle periods. |
| `slave_light_sleep_stop()` | Acquires the PM lock, forcing the CPU to stay at maximum frequency and preventing entry into sleep. |
| `slave_light_sleep_is_configured()` | Checks if the light sleep component is initialized and ready. |
| `slave_light_sleep_deinit()` | Cleans up resources, stops active sleep, and deletes PM locks. |

---

## 2. Mandatory Dependencies

Before the APIs can function, the underlying ESP-IDF and PHY components must be configured correctly.

* **Power Management** ([`CONFIG_PM_ENABLE`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/power_management.html)): Enables the base framework for CPU frequency scaling and sleep.
* **Tickless Idle** ([`CONFIG_FREERTOS_USE_TICKLESS_IDLE`](https://www.freertos.org/low-power-tickless-rtos.html)): Required for the RTOS to skip tick interrupts, allowing the CPU to enter a low-power state during idle.
* **MAC/BB Power Down** (`CONFIG_ESP_PHY_MAC_BB_PD`): **Strictly mandatory if Bluetooth/BLE is enabled.** Without this, the hardware will stay awake to maintain the radio, preventing light sleep.
* **Bluetooth Modem Sleep** (if Bluetooth is enabled):
  * **ESP32**: Enable `CONFIG_BTDM_CTRL_MODEM_SLEEP` (Component config → Bluetooth → Modem Sleep)
  * **ESP32-C3/C5/C6/C61/S3/H2**: Enable `CONFIG_BT_LE_SLEEP_ENABLE` (Component config → Bluetooth → Controller Options → Enable BLE sleep)

---

## 3. Power Saving Recommendations

For maximum power savings, consider these additional optimizations:

* **Disable Bluetooth if Unused**: If Bluetooth/BLE is not required, disable it entirely:
  * Via menuconfig: Component config → Bluetooth → [ ] Bluetooth
  * Via API: `esp_bt_controller_disable()`, `esp_bt_controller_deinit()`, `esp_bt_mem_release()`
  * This provides significant power savings by removing radio overhead.

* **WiFi Modem Sleep**: Enable WiFi power management to allow WiFi to sleep between packets:
  * Via menuconfig: Component config → Wi-Fi → WiFi Power Management
  * Via API: `esp_wifi_set_ps(WIFI_PS_MIN_MODEM)`
  * This allows WiFi to maintain connection while reducing power consumption.

* **WiFi ITWT (Immediate Target Wake Time)**: For Wi-Fi 6 (802.11ax) networks, enable ITWT for scheduled wake times:
  * Check example: [`examples/host_wifi_itwt`](https://github.com/espressif/esp-hosted-mcu/tree/main/examples/host_wifi_itwt)
  * Requires WiFi HE (802.11ax) support and AP support
  * Provides advanced power saving through scheduled wake times

---

## 4. Integration with Host Power Save

The automated integration hooks the Light Sleep APIs into the **[Host Power Save](https://github.com/espressif/esp-hosted-mcu/tree/main/docs/feature_host_power_save.md)** lifecycle. This ensures the slave only sleeps when the communication bus (SPI/SDIO/UART) is inactive.

### Hooking the Callbacks

The integration relies on four callbacks. You hook the `slave_light_sleep` APIs as follows:

1. **`host_power_save_on_prepare_cb()`** (Host Preparing to Sleep)
* **Slave Hook**: Application cleanup. Save state to NVS or flush data buffers while the bus is still **UP**.


2. **`host_power_save_on_ready_cb()`** (Host is Asleep)
* **Slave Hook**: Call **`slave_light_sleep_start()`**. Since the bus is now **DOWN**, it is safe to sleep.
* *Note*: If using Peripheral Powerdown, stop the CLI/UART here.


3. **`host_power_save_off_prepare_cb()`** (Host Wakeup Detected)
* **Slave Hook**: Call **`slave_light_sleep_stop()`**. This ensures the CPU is at full speed *before* the host re-initializes the bus.


4. **`host_power_save_off_ready_cb()`** (Host Fully Awake)
* **Slave Hook**: Resume services. Restart CLI/UART if they were stopped. The bus is now **UP**.



---

## 5. Configuration Tree & Options

Follow this tree structure in `eh.py menuconfig`. The tree illustrates how the Hosted sleep options depend on the base IDF components.

```text
# Slave side configuration: eh.py menuconfig

├── Component config
│    ├── Power Management
│    │   └── [*] Support for power management (CONFIG_PM_ENABLE)
│    │        └── [*] Power down Digital peripherals in light sleep (CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP)
│    │
│    ├── FreeRTOS
│    │   └── Kernel
│    │        └── [*] configUSE_TICKLESS_IDLE (CONFIG_FREERTOS_USE_TICKLESS_IDLE)
│    │
│    ├── PHY
│    │   └── [*] Power down MAC and baseband of Wi-Fi and Bluetooth... (CONFIG_ESP_PHY_MAC_BB_PD) (if Bluetooth is enabled)
│    │
│    └── Bluetooth (if Bluetooth is enabled)
│         ├── [*] Bluetooth (CONFIG_BT_ENABLED)
│         └── Controller Options
│              └── [*] Modem Sleep (ESP32 only: CONFIG_BTDM_CTRL_MODEM_SLEEP, C3/C5/C6/C61/S3/H2: CONFIG_BT_LE_SLEEP_ENABLE)
│
└── Example Configuration
     ├── [*] Allow host to power save
     │    └── Host Power Save Configuration
     │        └── [*] Unload low level BUS driver... (CONFIG_ESP_HOSTED_UNLOAD_BUS_DRIVER_DURING_HOST_SLEEP)
     │
     └── Light Sleep Power Management
          ├── [*] Enable light sleep power management (CONFIG_ESP_HOSTED_LIGHT_SLEEP_ENABLE)
          └── Light Sleep Parameters
              ├── (<frequency>) Minimum CPU frequency (MHz)
              └── [*] Power down peripherals (CONFIG_ESP_HOSTED_LIGHT_SLEEP_PERIPHERAL_POWERDOWN)

```

### Technical Ranges & Logic

* **Minimum CPU Frequency**: Sets the frequency when idle. Lower frequency = more power savings.
* *ESP32/S/C/H series*: 10 MHz - 240 MHz (C2/H2 max 80 MHz).
* *ESP32-C5*: 12 MHz - 96 MHz.


* **Peripheral Powerdown**: Enabling `CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP` allows digital peripherals (like SDIO & UART) to be powered off.
* *Note*: Requires `CONFIG_ESP_HOSTED_UNLOAD_BUS_DRIVER_DURING_HOST_SLEEP` to be enabled.
* *Trade-off*: UART console will be unavailable during sleep.

### Automated Config Checks

The `eh.py menuconfig` interface provides real-time feedback on your configuration:

* **✓ ESP-Hosted Light sleep allowed to configure**: All prerequisites are met, light sleep can be enabled.
* **✗ ESP-Hosted Light sleep not allowed**: Missing prerequisites are listed above this message.
* **⚠️ Prerequisite**: Missing mandatory configuration items are highlighted.
* **ⓘ Recommendation**: Optional optimizations for maximum power savings.
* **✓ Maximum Power Saving**: All optimizations are enabled when both bus driver unload and peripheral powerdown are configured.



---

## 6. Implementation Example

The system includes a reference implementation demonstrating the complete handshake.

* **Header**: [`slave/main/example_light_sleep.h`](https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/example_light_sleep.h)
* **Source**: [`slave/main/example_light_sleep.c`](https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/example_light_sleep.c)

### Manual Integration Snippet

If you are not using the automatic example (`CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_LIGHT_SLEEP`), manually hook the APIs in your `app_main()`:

```c
#include "slave_light_sleep.h"  // See: https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/slave_light_sleep.h
#include "host_power_save.h"     // See: https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/host_power_save.h

void app_main(void) {
    // 1. Initialize Light Sleep component
    slave_light_sleep_init();

    // 2. Setup Host Power Save Callbacks
    host_power_save_config_t ps_config = HOST_POWER_SAVE_DEFAULT_CONFIG();
    ps_config.callbacks = (host_power_save_callbacks_t) {
        .host_power_save_on_prepare_cb = my_app_cleanup,
        .host_power_save_on_ready_cb = slave_light_sleep_start,    // Enter Sleep
        .host_power_save_off_prepare_cb = slave_light_sleep_stop,  // Exit Sleep
        .host_power_save_off_ready_cb = my_app_resume
    };

    // 3. Start Host monitoring
    host_power_save_init(&ps_config);
}
```

---

## References

- [Host Power Save Documentation](https://github.com/espressif/esp-hosted-mcu/tree/main/docs/feature_host_power_save.md)
- [ESP-IDF Power Management](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/power_management.html)
- [FreeRTOS Tickless Idle](https://www.freertos.org/low-power-tickless-rtos.html)
- [Slave Light Sleep API](https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/slave_light_sleep.h)
- [Example Implementation](https://github.com/espressif/esp-hosted-mcu/tree/main/slave/main/example_light_sleep.c)
- [ESP-Hosted-MCU Repository](https://github.com/espressif/esp-hosted-mcu)
- [Slave Directory](https://github.com/espressif/esp-hosted-mcu/tree/main/slave)
