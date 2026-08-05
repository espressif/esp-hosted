# Unreleased

- host: fix memory leak by saving allocated GPIO ISRs in a list when enabled. GPIO ISR is deallocated from list and freed when disabled.

# Releases

# $${\color{green} \text{3.0.6}}$$

- bt: renamed the host BT stack adapter + API `esp_hosted_bt_stack*` → `esp_hosted_bt_host_stack*` (component, header, `setup()`/`teardown()`, `ESP_HOSTED_BT_HOST_STACK_CONFIG_*`); no back-compat
- bt (co-processor): Kconfig `ESP_HOSTED_BT_STACK_ENABLED` → `ESP_HOSTED_CP_BT_STACK_ENABLED`; no back-compat
- examples/bt: new `esp_hosted_custom/hci_demo` — bring-your-own BT stack (raw HCI via the custom path) showcase + test
- docs: new public host API reference — [`docs/api-reference.md`](api-reference.md)
- ci: premerge gate ran a missing `.py`; now `bash tools/check_override_path_depth.sh`

# $${\color{green} \text{3.0.5}}$$

- host: drop the backward `esp_wifi_remote` dependency — its header now resolves transitively via `esp_wifi` (`app -> esp_wifi_remote -> esp_hosted`); non-ESP hosts unchanged
- host/power: use `ESP_EXT1_WAKEUP_ANY_LOW` on post-ESP32 targets; keep `ALL_LOW` on ESP32

# $${\color{green} \text{3.0.4}}$$

- forbid enabling `esp_hosted` when CONFIG_ESP_HOST_WIFI_ENABLED=y
- fixed Wi-Fi DPP feature on co-processor
- **note:** DPP works only on ESP-IDF v5.5 and above with no or NULL supplicant callback

# $${\color{green} \text{3.0.3}}$$

- IDF 6.1+: skip PRIVATE-linking the source-less `esp_wifi_remote` shim (was a fatal configure error)
- fix auto-init worker-task leak across CP deinit/re-init cycles
- `shut_down_cp_when_unused` example: drop host power-save config
- light-sleep example: gate peripheral power-down on SoC support
- clear `esp_wifi_remote` channels on deinit; wakeup-GPIO pull matches idle level
- add 10-cycle CP cold-boot soak test (heap-poisoned)

# $${\color{green} \text{3.0.2}}$$

- dev-board support: P4x-C5, P4-EYE, M5Stack Tab5, and P4 C5/C6/C61 core boards (per-board GPIO defaults)
- iTWT auto-enabled on Wi-Fi 6 coprocessors; iTWT console commands added to the api_exerciser
- registry publish: strip bundled local-component deps so examples resolve from the copied `components/`
- pre-commit: added codespell spell-check; version header auto-syncs from `idf_component.yml`

# $${\color{green} \text{3.0.1}}$$

- registry cp/host example READMEs generated from the top README at publish time
- added `wifi/iperf` coprocessor example
- network-split examples use the shared `esp_hosted_examples_common` helper
- board-agnostic example configs; dropped orphaned `sdkconfig.ci*` files
- idempotent event-loop / netif init with default-netif wrappers
- feature-descriptor linker placement fixes (ldgen, section alignment)
- SDIO SW_AGGR build fails early on an unpatched ESP-IDF
- UART and SPI-FD default the CP reset line to active-low
- network-split code gated on the host feature, not a CP-only symbol
- renamed board choice `ESP_HOST_DEV_BOARD` to `ESP_HOSTED_DEV_BOARD`
- silenced benign multiple-definition report; dropped dead BT-UART symbols
- emu test parity (spi_fd, OTA); physical-core-aware `--jobs`
- default to ESP-IDF v6.0.2
- portable semaphore semantics (0 = poll, WAIT_FOREVER = block)


# $${\color{cyan} \text{3.0.0}}$$

### Migration and compatibility

- unified the supported MCU and Linux host/co-processor stacks
- added a migration guide for MCU 2.x release candidates and Linux FG 1.x/2.x
- added build-time legacy Linux compatibility images:
  - FG 1.x: `CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V1`
  - FG 2.x: `CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V2`
- current Linux and MCU builds use RPC-V2; legacy Linux compatibility images
  use RPC-V1 (`ctrlResp` / `ctrlEvnt`) and legacy wire interface IDs
- ESP-Hosted-FG is fully migrated; ESP-Hosted-NG is not fully migrated

### Release documentation

- added migration, pending-items, and license-inventory documentation

### ESP-IDF setup

- `set-idf-path <dir>` reuses that SDK, checks patch state, and directs the user to `patch-idf` before a co-processor build when needed


# $${\color{green} \text{2.12.11}}$$

- fixed SDIO deinit tearing down the shared SDMMC host (broke a co-existing SD card on the other slot)
- fixed public headers failing to build under `-Werror=undef` (e.g. Matter/external host consumers)

# $${\color{green} \text{2.12.10}}$$

- added `esp_wifi_disable_pmf_config()` support (host ↔ co-processor RPC); previously returned `ESP_ERR_NOT_SUPPORTED`
- fixed SDIO RX heap corruption on transport deinit and hardened RX buffer allocation

# $${\color{green} \text{2.12.9}}$$

- added gpios for `ESP32_P4X_C5_Function_EV_Board V2.0`
- fix build break for IDF v6 when enable power save (API renamed)
- update Kconfig to check for config `SOC_GPIO_SUPPORT_HP_PERIPH_PD_SLEEP_WAKEUP` introduced in IDF v6
- fixed bug causing a crash in SPI-HD interface with IDF v6.1 due to new uninitialised member in `spi_bus_config_t`
- reduced default number of buffers for SPI-HD and SPI-FD to resolve memory issues
- Zigbee:
  - added support for Zigbee
  - added Home Automation thermostat on a Zigbee Coordinator example
- added support for SPI-HD 1-bit mode (SPI 3-wire interface)
  - **NOTE**: SPI-HD 1-bit mode is only supported on ESP-IDF v6.1 and above and requires [git Commit bf10423](https://github.com/espressif/esp-idf/commit/bf10423a5b01888b33ab1f4e45ef5a880eed69e6)
- updated `_h_get_semaphore` and `_h_lock_mutex` to use milliseconds instead of seconds as a timeout parameter
- added `_h_thread_yield` for use by threads to request a context switch

# $${\color{green} \text{2.12.8}}$$

- SDIO: added `ESP_HOSTED_MEMPOOL_PREFER_SPIRAM` to allocate transport buffers from PSRAM (e.g. ESP32-P4), saving internal RAM; off by default
- OS APIs: `_h_get_semaphore` and `_h_lock_mutex` now take timeout in milliseconds (was seconds)
- added `_h_thread_yield` for threads to request a context switch

# $${\color{green} \text{2.12.7}}$$

- OpenThread: added OpenThread over dedicated UART support
  - co-processor is the OpenThead RCP (Radio Co-Processor)
  - added host examples: `host_openthread_border_router`, `host_openthread_cli`
- Common Mempool: fixed build error on ESP-IDF v6.x when using PicolibC with `CONFIG_LIBC_PICOLIBC_NEWLIB_COMPATIBILITY` disabled
- Host: removed Unicode encoded characters in cmake file to prevent Windows build failure

# $${\color{green} \text{2.12.6}}$$

## Bug Fixes

- make `TAG` in `mempool.c` static to avoid link-time clash with other components (#187)

# $${\color{green} \text{2.12.5}}$$

## Features

### External Coexistence: allow with BT on advanced coex chips
- Aligned Kconfig with IDF change that relaxes `ESP_COEX_EXTERNAL_COEXIST_ENABLE` dependency
- On chips with `SOC_EXTERNAL_COEX_ADVANCE`, external coexistence now works alongside BT controller
- Updated compile-time checks in `slave_ext_coex.h` to match (includes `soc/soc_caps.h`)

## Bug Fixes

- fixed CI to allow building ESP32 co-processor with ESP-IDF v5.5 for SPI-FD and UART transports: was running out of IRAM space
- fixed CI build failure when building co-processor with ESP-IDF release/v5.3
- added more ESP-IDF releases to CI for testing

### OTA: fix image size calculation for partition-based OTA
- Add 16-byte alignment padding before SHA256 hash in image size parser
- Previously sent 15 fewer bytes than actual image, causing hash mismatch

# $${\color{green} \text{2.12.4}}$$

## Bug Fixes

- fix build break on co-processor if using SPI-HD interface with 2 data lines

## Features

### Custom RPC callbacks: support user context pointer

  - Allows passing per-callback context without global state
  - User pointer is returned as-is on every invocation

##### API Changes

- `esp_hosted_register_custom_callback`

```c
// Old
esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id,
    void (*callback)(uint32_t msg_id, const uint8_t *data, size_t data_len));

// New
esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id,
    void (*callback)(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user),
    void *user);
```

### Others

- used common mempool code for both Host and Co-processor
- made ESP-Hosted mempool code private to fix build break
- added parameter checking for RPC calls

## Bug Fixes

- Host: added NULL or validation checks for exposed user APIs

# $${\color{green} \text{2.12.3}}$$

## Bug Fixes

- Fixed `esp_wifi_scan_get_ap_records` to set the actual AP number this API returns

# $${\color{green} \text{2.12.2}}$$

## Bug Fixes

- Add slave target strings to `Kconfig` (required by Arduino build)

## Features

- Add api to get the co-processor name and chip id to identify the co-processor
- API Added
  - `esp_hosted_get_cp_info`
- Updated `examples/host_bt_controller_mac_addr` to request this information
- Extended RPC for GetCoprocessorFwVersion to include the co-processor name and chip id

# $${\color{green} \text{2.12.1}}$$

## Features

### Allow disabling Wi-Fi and/or Bluetooth

- Added Co-processor option to disable Wi-Fi support (for Bluetooth-only support)
- Re-organised co-processor code: moved Wi-Fi, Wi-Fi Enterprise and Network Split code into individual files
- Added initial support for ESP32-H4 as co-processor
  - only works with UART interface as ESP-Hosted Transport. SPI to be enabled later.
  - Bluetooth not yet enabled in ESP-IDF

### Co-processor: External Coexistence

- Add support to manage co-processor external Wi-Fi coexistence from the host.
- APIs Added
  - `esp_hosted_cp_ext_coex_set_work_mode`
  - `esp_hosted_cp_ext_coex_set_gpio_pin`
  - `esp_hosted_cp_ext_coex_set_grant_delay`
  - `esp_hosted_cp_ext_coex_set_validate_high`
  - `esp_hosted_cp_ext_coex_disable`
- Host Example Added
  - examples/host_manage_copro_ext_coex
- Documentation
  - examples/host_manage_copro_ext_coex/README.md
- Config
  - Host
    - ESP_HOSTED_CP_EXT_COEX
    - ESP_HOSTED_CP_EXT_COEX_ADVANCE
  - Slave
    - ESP_HOSTED_CP_EXT_COEX

### Other Features

- Added support for Bluetooth-only Co-processors (like ESP32-H2)
- Allow ESP-Hosted component to be manually disabled through idf menuconfig

## Bug Fixes

- ESP Slave FW validation fails because OTA image validation depends on non-existent IDF 6.0+ APIs (GitHub ##165)

# $${\color{cyan} \text{2.12.0}}$$

## Features

- Checked incoming image validity during OTA update
  - done for ESP-IDF v6.1.0 or greater
- Wi-Fi APIs
  - `esp_wifi_set_scan_parameters()`
  - `esp_wifi_get_scan_parameters()`
- Allow slave OTA only if correct SPI Flash Mode

## Bug Fixes

- Assert if slave uses SDIO streaming and host as SDIO packet mode
- Guard `esp_hosted_coprocessor.h`, `host_power_save.h`, `interface.h` for cplusplus inclusion
- Replace assert with graceful error on mempool alloc failure
- Building with and without bt enabled
- Disable auto connect upon sta mode started
- Add `esp_eap_client_set_eap_methods()` as `weak` in `esp_wifi_weak.c`

## Config

- Added ESP32-P4-Eye board GPIOs at slave and host Kconfig
- Fix host wakeup GPIO config

# $${\color{green} \text{2.11.7}}$$

## Features

- Added Co-processor Memory Monitor: sets up a heap memory monitor on the co-processor that periodically checks the amount of heap memory remaining. Co-processor sends memory info events to the host at periodic intervals or when heap memory falls below memory thresholds set by the host.
- Added `examples/host_hosted_cp_meminfo` as an example:
  - make a one time request of heap memory
  - request periodic memory reports
  - get a report only when heap memory falls below a threshold

## API Added

- `esp_hosted_set_mem_monitor`

## Event Added

- `ESP_HOSTED_EVENT_MEM_MONITOR`

## Documentation

Added performance with ESP32-C3 as co-processor, using SPI-FD interface, to Performance document.

# $${\color{green} \text{2.11.6}}$$

- Fix a build break on ESP-IDF master branch
- Add `ESP_HOSTED_WIFI_AUTO_CONNECT_ON_STA_START` to control whether WiFi station auto-connects on STA start on both host and slave sides. This allows disabling auto-connect to align behavior with standard ESP-IDF examples and avoids unintended connection attempts during initialization.
- Made FreeRTOS runtime stats logging optional
- Added option to allow slave to reuse application-created STA netif handle instead of creating its own handle
- Added option to disable sharing Bluetooth with Host, for cases where BT is only required on the co-processor

# $${\color{green} \text{2.11.5}}$$

## Bug Fixes

- Renamed `H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT` to `H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT_MS` to clarify units are in milliseconds.

# $${\color{green} \text{2.11.4}}$$

## Features

- ESP32-P4 C61 Core board support - Improvise

## Bug Fixes

- TCP iPerf stability with documented performance optimizations

## Tested

- Host power save and wake-up functionality
- Wake-up GPIOs:
  - P4 Core Board – C61: IO04
  - P4 Core Board – P4: IO06
  - GPIOs disabled by default (`-1`) due to no physical connection; verified via jumper wiring and solder
- Network split scenarios

# $${\color{green} \text{2.11.3}}$$

## Bug Fixes

- made UART Hosted interface more stable:
  - flush the input after reset. Rx line may toggle while resetting the co-processor, causing Host UART to store invalid data.
  - check that offset in received payload header is valid: discard packet for invalid offsets.
  - check flags in received payload only after the payload is considered valid

# $${\color{green} \text{2.11.2}}$$

Minor fix: On Timeout/Failure, Print RPC req str instead of RPCId

# $${\color{green} \text{2.11.1}}$$

Minor fixes: const qualifier violations while building

# $${\color{cyan} \text{2.11.0}}$$

## Bug Fixes

- remove double freeing of buffer if `chan_arr[buf_handle->if_type]->rx()` fails. Underlying rx function will free the memory

> [!WARNING]
> This version of ESP-Hosted onwards must be used with wifi-remote component v1.3.1 or greater. See the [Migration Guide](docs/migration.md) for more information.

# $${\color{cyan} \text{2.10.0}}$$

## Features: GPIO Expander

- **GPIO Expander**: Added feature to allow the host to control the GPIOs of the slave co-processor over the existing transport link. See [GPIO Expander Guide](./docs/gpio_expander.md).
- **GPIO Expander Example**: Added a new example `examples/host_gpio_expander` to demonstrate the usage of the GPIO expander feature.

## APIs Added

- `esp_hosted_cp_gpio_config`
- `esp_hosted_cp_gpio_reset_pin`
- `esp_hosted_cp_gpio_set_level`
- `esp_hosted_cp_gpio_get_level`
- `esp_hosted_cp_gpio_set_direction`
- `esp_hosted_cp_gpio_input_enable`
- `esp_hosted_cp_gpio_set_pull_mode`

# $${\color{green} \text{2.9.7}}$$

## Features

- Add example, [host_shuts_down_slave_to_power_save](https://components.espressif.com/components/espressif/esp_hosted/examples/host_shuts_down_slave_to_power_save)
  - Use `EN` pin on coprocessor to power off/on
  - Power down coprocessor when not in use
  - Power on coprocessor when required
  - Connect Wi-Fi on coprocessor wake up

## Bug Fixes

- Fix the memory leaks in hosted deinit -> init path

# $${\color{green} \text{2.9.5 - 2.9.6}}$$

Using shorter, more manageable names for esp hosted events
Before adoption, concise the event names from full string COPROCESSOR to just CP in event names

# $${\color{green} \text{2.9.4}}$$

## Features

- enabled ESP-Hosted events. Host can register an event handler to receive these events from co-processor:
  - INIT event, indicating the co-processor has started
  - HEARTBEAT event, when enabled by the host
  - TRANSPORT_FAILURE event, when ESP-Hosted encounters a transport failure
  - host can use these events to determine if the co-processor rebooted (unexpected INIT event) or hanged (missing HEARTBEAT)
- added `examples/host_hosted_events` as an example to show how the host can use either event to reinitialise a Station connection to an AP

## Bug Fixes

- fixed ESP-Hosted and SDIO issues that prevent transport reinitialisation
- fixed files to skip when running codespell during pre-commit

# $${\color{green} \text{2.9.3}}$$

## Bug Fixes

- removed setting `scan_method` and `sort_method` in co-processor when station is connecting. Use the values sent by the Host in the Wi-Fi config.
- fixed support for ESP32-S2 as a co-processor

# $${\color{green} \text{2.9.2}}$$

## Bug Fixes

- Slave OTA Example
  - Add version-aware OTA activation
  - Conditionally call esp_slave_ota_activate() only for slave FW >= v2.6.0
- Improved Slave OTA Documentation
  - Comprehensive code comments explaining OTA APIs and version checks
  - Mermaid sequence diagram showing complete OTA verification flow

# $${\color{green} \text{2.9.1}}$$

## Bug Fixes

- Correct esptool command usage (`write_flash` instead of invalid `write-flash`)
- Update Wi-Fi bandwidth enums for newer ESP-IDF compatibility

## Improvements

- Better validation and user-readable error messages for slave OTA
- Detect empty or uninitialized LittleFS and partition OTA sources
- Clear guidance when invalid or missing slave firmware binaries are detected

# $${\color{green} \text{2.9.0}}$$

## Bug Fixes

- Fix slave OTA failures on back-to-back updates by removing the hard dependency on `CONFIG_ESPTOOLPY_FLASHMODE_QIO`.
- Previously, mandating QIO flash mode caused consecutive OTA operations to fail; this is now resolved by removing the forced flash mode setting.

# $${\color{red} \text{2.8.5}}$$

## Features: Light Sleep Integration & Documentation

- **Light Sleep Documentation**: Added light-sleep integration guidance for the host/co-processor handshake.
- **Smart Wakeup Demo**: Added a new demo showcasing the slave waking the host MCU using specific network triggers (UDP packets).
  - Example: `examples/host_network_split__power_save/host_wakeup_demo_using_udp_packet`

## Bug Fixes & Stability

- **Memory Leak Fixes**:
  - Resolved memory leaks in the SDIO driver during unload/deinit by ensuring proper buffer cleanup and `sdio_slave_send_get_finished` usage.
  - Fixed memory leaks in `esp_hosted_cli` by adding proper deregistration APIs for Hosted-specific commands.
  - Cleanly handled timer memory removal in the Host Power Save component.
- **Concurrency & Race Conditions**:
  - Added semaphore protection (mutex) in `host_power_save.c` to prevent race conditions when multiple threads attempt to wake the host simultaneously.
- **Timing & Reset Improvements**:
  - Adjusted task delays in the host wakeup sequence to prevent the host from receiving incorrect or premature reset signals during the wake-up transition.

---

# $${\color{red} \text{2.8.4}}$$

## Features: Slave Auto Light sleep

- Auto Invoked when host triggers deep sleep
- Example implementation in example_light_sleep.c

# Known issue

There is memory leak when sdio driver unload is selected - working on the fix

# $${\color{red} \text{2.8.3}}$$

Add up mutex protection for callback array for Peer Data Transfer

# $${\color{red} \text{2.8.2}}$$

## Amend Peer Data Transfer example with custom msg id

Amend [Peer Data Transfer Example](https://components.espressif.com/components/espressif/esp_hosted/examples/host_peer_data_transfer):

Features:
- User can register callback-based dispatch for their own msg ids, both at host and slave
- Configurable handler slots via Kconfig (default: 3)

API:
- esp_err_t esp_hosted_send_custom_data(uint32_t msg_id, const uint8_t *data, size_t data_len)
- esp_err_t esp_hosted_register_cu
                                  stom_callback(uint32_t msg_id, void (*callback)(uint32_t msg_id, const uint8_t *data, size_t data_len));

Example (examples/host_peer_data_transfer):
- Uses animal sound theme (CAT→MEOW, DOG→WOOF, HUMAN→HELLO)
- Host sends CAT_MSG_ID, with byte stream. Slave sends back same stream with MEOW_MGD_ID and so on.

Configuration:
- Host: CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS (Kconfig)
- Slave: CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS (Kconfig.projbuild)
- Ported as H_MAX_CUSTOM_MSG_HANDLERS on host side

## Allow to disable app_main() from slave

**Kconfig : `CONFIG_ESP_HOSTED_COPROCESSOR_APP_MAIN`** at coprocessor menuconfig
- **Default**: Enabled (for slave example from registry)
- **Purpose**: Controls whether ESP-Hosted provides its own `app_main()`
- **When to disable**:
  - Using ESP-Hosted slave code base as a component in your application

# $${\color{red} \text{2.8.1}}$$

## Example: [Peer Data Transfer Example](https://components.espressif.com/components/espressif/esp_hosted/examples/host_peer_data_transfer)

## Features

- Supports sending and receiving arbitrary (preformatted) user data from/to host and slave
- Maximum payload size: 8166 bytes per packet

## APIs

- `esp_hosted_send_custom_data(data, length)`: Send raw binary data to coprocessor
- `esp_hosted_register_rx_callback_custom_data(callback)`: Register callback for receiving custom data

# $${\color{red} \text{2.8.0}}$$ - Network Split (Shared IP)

**Network Split (Shared IP)**

This major update allows the **Host MCU** and the **ESP Slave** to share a **single IP address** while running independent network stacks. This is ideal for low-power products where the Slave handles background tasks while the Host sleeps.

- Disabled by default. Enable using `CONFIG_ESP_HOSTED_NETWORK_SPLIT_ENABLED` config
- **Documentation**: [Network Split Guide](docs/features/network-split.md).

## Example: [Network Split with Host Power Save Example](https://components.espressif.com/components/espressif/esp_hosted/examples/host_network_split__power_save)

## **Key Additions**

- Smart Traffic Routing: `nw_split_router.c` automatically directs traffic through function, `nw_split_filter_and_route_packet()`
- **Port-based (TCP/UDP)**: Uses `CONFIG_LWIP_TCP_LOCAL_PORT_RANGE` and `CONFIG_LWIP_UDP_LOCAL_PORT_RANGE` to decide if the Host or Slave handles a packet.
- **Reserved Ports**: Mandate packets on specific ports (e.g., 80, 443) to the Host via `CONFIG_ESP_HOSTED_HOST_RESERVED_PORTS_CONFIGURED`.
- **Non TCP/UDP**: Built-in handling for `ARP`, `ICMP`, and `DHCP` on coprocessor (configurable) to offload host for other priority work or deep sleep
- iperf Performance
  - Demo of sharing same port: Port `5001` is shared smartly, allowing performance testing on either stack (at a time) without reconfiguring.
- Low-Power Support
  - (Optionally) Deeply integrated with [Host Power Save](docs/features/power-save.md)
- Smart Wakeup demo
  - The Slave can "wake up" the Host when it receives specific traffic or an MQTT message containing the `"wakeup-host"` string.
- **Collision Prevention**
  - Added `esp_hosted_lwip_src_port_hook.h` to ensure the Host and Slave never try to use the same source port.
- **Supported Targets**
  - **Slaves**: ESP32-C5, C6, S2, S3.
  - **Hosts**: ESP32-P4, H2, and non ESP MCUs.

ESP Component Registry Release: [2.8.0](https://components.espressif.com/components/espressif/esp_hosted/versions/2.8.0)

# $${\color{red} \text{2.7.4}}$$

## Bug Fixes

- fixed co-processor to properly allow wifi init and deinit
- fixed registration of event handlers in co-processor

# $${\color{red} \text{2.7.3}}$$

## Bug Fixes

- fixed RPC Response for OTA commands to return errors in responses correctly
- fixed double free bug in host OTA example

# $${\color{red} \text{2.7.2}}$$

## Bug Fixes

- Stable workaround for ota writes to slave

# $${\color{red} \text{2.7.1}}$$

- Add support for more PCBs:
  - ESP32-P4 Core Board - with on-board C5
  - ESP32-P4 Core Board - with on-board C6

# $${\color{red} \text{2.7.0}}$$

## Bug Fixes

Restructured the predecessor MCU commits.

# $${\color{red} \text{2.6.8}}$$

## Bug Fixes

- Clean up ESP-Hosted prints at host

# $${\color{red} \text{2.6.7}}$$

## Features

- Added Hosted API call to get co-processor Application Descriptor

# $${\color{red} \text{2.6.6}}$$

## Bug Fixes

- Fixed sta connection to remove extra disconnected event if incoming station config is different from current station config
- IRAM size limitation when using UART transport only applies to ESP32, not to all SOCs.
- workaround a bug in `esp_wifi_get_protocol()` that can cause memory corruption. See this [ESP-IDF Issue](https://github.com/espressif/esp-idf/issues/17502).
- updated CI pipelines to build mqtt/tcp example from Registry Component on master branch

# $${\color{red} \text{2.6.5}}$$

## Features

- Add example showing concurrent use of a SD Card and ESP-Hosted.

# $${\color{red} \text{2.6.4}}$$

## Bug Fixes

- Fix the `esp_wifi_deinit()` call from host

# $${\color{red} \text{2.6.3}}$$

## Bug Fixes

- Increase timing used to reset co-processors to work with a slower FreeRTOS clock tick
- Updated documentation on performance optimization

# $${\color{red} \text{2.6.2}}$$

## Bug Fixes

- fixed bug in enabling `esp_eap_client_set_eap_methods` on co-processor based on ESP-IDF version

# $${\color{red} \text{2.6.1}}$$

## Bug Fixes

Minor fixes in Slave OTA example

# $${\color{red} \text{2.6.0}}$$

- Added public OTA APIs for slave firmware updates
- Added host-triggered slave OTA example with support for HTTP, partition, and filesystem sources
- Support for LittleFS filesystem-based OTA updates
- Migration guide updated for 2.6.0

## APIs added

- `esp_hosted_ota_begin`
- `esp_hosted_ota_write`
- `esp_hosted_ota_end`
- `esp_hosted_ota_activate`

## APIs deprecated

- `esp_hosted_slave_ota` - Use the new [Host Performs Slave OTA Example](examples/host_performs_slave_ota/README.md) instead for more flexible OTA implementations with comprehensive documentation and multiple deployment methods

## Examples added

- `host_performs_slave_ota` - Host-triggered slave OTA example supporting HTTP URLs, partition sources and LittleFS filesystem sources

# $${\color{red} \text{2.5.12}}$$

## Features

- Add SPI (full and half duplex) and UART support for ESP32-C61
- Updated documentation on applying optimised Wi-Fi settings to sdkconfigs

## Bug Fixes

- Fixed build issues when raw throughput testing is enabled
- Fixed bug in co-processor causing SDIO to operate only in packet mode

# $${\color{red} \text{2.5.11}}$$

## Bug Fixes

- Fixes to use compatible version of `idf-build-apps` and constraints during CI pipeline builds
- Renamed CI pipelines to "sanity" and "regression"
- Prefix jobs with `sanity_` or `regression_` to make their names unique
- Enabled building of ESP-Hosted examples in regression pipeline
- Various bug fixes found in the process of fixing the CI pipelines

# $${\color{red} \text{2.5.10}}$$

## Features

- Version, 2.5.8 - 2.5.10:
  - Add staging branch workflow for safer component releases

# $${\color{red} \text{2.5.7}}$$

## Bug Fixes

- Fixed build break when CLI Commands are enabled on coprocessor

# $${\color{red} \text{2.5.6}}$$

## Bug Fixes

- Updated co-processor and some example `idf_component.yml` files to set component dependencies based on the ESP-IDF version in use

# $${\color{red} \text{2.5.5}}$$

## Bug Fixes

- Fixed build errors when using latest version of ESP-IDF
- Updated Wi-Fi Easy Connect (DPP) code to match current ESP-IDF master
- Adjusted CI pipeline

# $${\color{red} \text{2.5.4}}$$

## Features

- Added building with ESP-IDF v5.3 in CI
- Added building ESP-Hosted examples in CI

## Bug Fixes

- Fixed building with ESP32-H2 as host in CI (was skipping build)

# $${\color{red} \text{2.5.3}}$$

## Bug Fixes

- Fix the ESP-IDF CI

# $${\color{red} \text{2.5.2}}$$

## Features

- Add support to get and set the BT Controller Mac Address
- To support set BT Controller Mac Address, BT Controller is now disabled by default on the co-processor, and host must enable the BT Controller. See [Initializing the Bluetooth Controller](docs/features/bluetooth.md#31-initializing-the-bluetooth-controller) for details
- Updated all ESP-Hosted BT related examples to account for new BT Controller behaviour

## APIs added

- `esp_hosted_bt_controller_init`
- `esp_hosted_bt_controller_deinit`
- `esp_hosted_bt_controller_enable`
- `esp_hosted_bt_controller_disable`
- `esp_hosted_iface_mac_addr_set`
- `esp_hosted_iface_mac_addr_get`
- `esp_hosted_iface_mac_addr_len_get`

# $${\color{red} \text{2.5.1}}$$

## Bug Fixes

- Added dependency on `esp_driver_gpio`

# $${\color{red} \text{2.5.0}}$$

## Bug Fixes

- Remove dependency on deprecated `driver` component and added necessary dependencies instead

# $${\color{red} \text{2.4.3}}$$

## Features

- Add support for Wi-Fi Easy Connect (DPP)
- [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_dpp.html) on Wi-Fi Easy Connect (DPP)
- [ESP-Hosted Enrollee Example](examples/wifi/dpp/README.md) using DPP to securely onboard a ESP32P4 with C6 board to a network with the help of a QR code and an Android 10+ device

## APIs added

- `esp_supp_dpp_init`
- `esp_supp_dpp_deinit`
- `esp_supp_dpp_bootstrap_gen`
- `esp_supp_dpp_start_listen`
- `esp_supp_dpp_stop_listen`

# $${\color{red} \text{2.4.2}}$$

## Bug Fixes

- Fix ignored lwip hook header in slave example

# $${\color{red} \text{2.4.1}}$$

## Bug Fixes

- Reduced ESP32 bootloader size

# $${\color{red} \text{2.4.0}}$$

## Features

- Added support for Wi-Fi Enterprise

## APIs added

- `esp_wifi_sta_enterprise_enable`
- `esp_wifi_sta_enterprise_disable`
- `esp_eap_client_set_identity`
- `esp_eap_client_clear_identity`
- `esp_eap_client_set_username`
- `esp_eap_client_clear_username`
- `esp_eap_client_set_password`
- `esp_eap_client_clear_password`
- `esp_eap_client_set_new_password`
- `esp_eap_client_clear_new_password`
- `esp_eap_client_set_ca_cert`
- `esp_eap_client_clear_ca_cert`
- `esp_eap_client_set_certificate_and_key`
- `esp_eap_client_clear_certificate_and_key`
- `esp_eap_client_set_disable_time_check`
- `esp_eap_client_get_disable_time_check`
- `esp_eap_client_set_ttls_phase2_method`
- `esp_eap_client_set_suiteb_192bit_certification`
- `esp_eap_client_set_pac_file`
- `esp_eap_client_set_fast_params`
- `esp_eap_client_use_default_cert_bundle`
- `esp_wifi_set_okc_support`
- `esp_eap_client_set_domain_name`
- `esp_eap_client_set_eap_methods`

# $${\color{red} \text{2.3.3}}$$

## Features

- Added SDIO support for ESP32-C61

# $${\color{red} \text{2.3.2}}$$

## Features

- Add host example to showcase transport config before `esp_hosted_init()`

# $${\color{red} \text{2.3.1}}$$

## Bug Fixes

- Fixed a build break caused by refactoring

# $${\color{red} \text{2.3.0}}$$

## Features

- Refactored common and port specific code

# $${\color{red} \text{2.2.4}}$$

## Bug Fixes

- Fixed SPI Full Duplex startup sequence
- Fixed incorrect Handshake GPIO assignment for C5 on Module
- Added valid CPU frequencies in ITWT Example for H2

# $${\color{red} \text{2.2.3}}$$

## Bug Fixes

- Fixed itwt build break for IDF v5.3.1

# $${\color{red} \text{2.2.2}}$$

## Features

- Added support for Wi-Fi Power Save and ITWT
- Added ITWT example
- Updated copyright check to allow Unlicensed or CC0-1.0 files

## APIs added

- `esp_wifi_set_inactive_time()`
- `esp_wifi_get_inactive_time()`
- `esp_wifi_sta_twt_config()`
- `esp_wifi_sta_itwt_setup()`
- `esp_wifi_sta_itwt_teardown()`
- `esp_wifi_sta_itwt_suspend()`
- `esp_wifi_sta_itwt_get_flow_id_status()`
- `esp_wifi_sta_itwt_send_probe_req()`
- `esp_wifi_sta_itwt_set_target_wake_time_offset()`

# $${\color{red} \text{2.2.1}}$$

## Features

- Allow external code to override Hosted BT Tx function by making it a `weak` reference

# $${\color{red} \text{2.2.0}}$$

## Features

- Add support for fragmentation of packets from sdio host to slave

# $${\color{red} \text{2.1.11}}$$

## Bug Fixes

- Fixed SoftAP operation issues
