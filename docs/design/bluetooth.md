# Bluetooth Design

[Home](../../README.md) · [Getting Started: Linux](../getting-started-linux.md) · [Getting Started: MCU](../getting-started-mcu.md) · [Troubleshooting](../troubleshooting.md)

ESP-Hosted carries Bluetooth HCI packets between a **Bluetooth host stack on the host MCU** and a **Bluetooth controller on the co-processor**. The host runs the app and the host stack; the co-processor runs the controller and radio. Hosted is stack-agnostic — the reference examples use both `esp-nimble` and `esp-bluedroid`, and you can port your own stack with modest effort.

> [!IMPORTANT]
> Classic Bluetooth needs an **ESP32** co-processor. As of today ESP32 supports Classic-BT + BLE; the other ESP chipsets are BLE-only. If your co-processor is an ESP32, note it only supports Bluetooth v4.2, so the host stack must also be v4.2.

---

## The pieces

### Bluetooth controller

Hosted uses the Bluetooth controller running on the co-processor and is just the communication medium, so it supports both Classic BT and BLE controllers — whichever the chosen chipset provides.

### Choosing a host stack

- **BlueDroid** — supports Classic Bluetooth *and* BLE. Use it for any Classic BT use case. `esp-bluedroid` is a fork of the Bluedroid-based stack shipped with ESP-IDF.
- **NimBLE** — BLE only, with a smaller code and RAM footprint. Recommended for BLE-only use cases. `esp-nimble` is a fork of Apache NimBLE shipped with ESP-IDF.

Confirm the memory requirement of your chosen host stack can be met on the host.

---

## Bluetooth interface: two transport options

Hosted offers two ways for the host stack to reach the controller: **Standard HCI** and **Hosted HCI**.

| | **Hosted HCI** | **Standard HCI (UART)** |
| :--- | :--- | :--- |
| What it is | Standard HCI encapsulated with an ESP-Hosted header | Transparent standard HCI, no extra metadata |
| Transport | Multiplexed over the existing ESP-Hosted bus (SPI/SDIO) | Dedicated transport (e.g. UART); cannot multiplex other traffic |
| Extra GPIOs | None beyond the ESP-Hosted interface | Extra GPIOs for the HCI interface |
| Setup | Easiest — once ESP-Hosted transport is up, Bluetooth just works | Portable — swap in any co-processor with a BT controller |
| Pick it for | Control, debugging flexibility, fewer GPIOs | Transparency and portability |

**Standard HCI** works in dedicated mode: HCI frames originating from the host stack are sent through an interface such as UART directly to the controller. Because it is plain HCI, you can replace the co-processor with any other (ESP or otherwise) that has a BT controller.

**Hosted HCI** embeds the ESP-Hosted header and reuses the underlying transport, multiplexing Bluetooth alongside other interface types on the same bus.

> [!NOTE]
> If Hosted HCI is configured as the Bluetooth transport, your Standard-HCI configuration must be disabled — and vice versa.

---

## Initializing the Bluetooth controller

> [!NOTE]
> Before ESP-Hosted-MCU v2.5.2 the controller was **enabled** by default. The steps below apply to **v2.5.2 and later**, where the controller is **disabled** by default so the BT MAC address can be set before the controller comes up.

**Get / set the BT MAC address:**

1. `esp_hosted_connect_to_slave()` — initialise the transport.
2. *(optional)* `esp_hosted_iface_mac_addr_len_get` — get the BT MAC address length.
3. *(optional)* `esp_hosted_iface_mac_addr_get` — read the current BT MAC address.
4. `esp_hosted_iface_mac_addr_set` — set the BT MAC address.

> [!NOTE]
> This MAC setting is temporary and reverts on device reset. For a permanent change, set it during hardware provisioning or burn it into eFuse.

**Enable the controller:**

1. `esp_hosted_connect_to_slave()` — initialise the transport (skip if already done above).
2. *(optional)* `esp_hosted_iface_mac_addr_get()` / `esp_hosted_iface_mac_addr_set()`.
3. `esp_hosted_bt_controller_init()`.
4. `esp_hosted_bt_controller_enable()`.
5. Initialise the BT host stack.

**Disable the controller:**

1. Deinitialise the BT host stack.
2. `esp_hosted_bt_controller_disable()`.
3. `esp_hosted_bt_controller_deinit()`.

---

## NimBLE host stack

The NimBLE port implements the NimBLE transport API set: `ble_transport_ll_init`, `ble_transport_to_ll_acl_impl`, `ble_transport_to_ll_cmd_impl`, `ble_transport_to_hs_evt`, `ble_transport_to_hs_acl` (see `examples/common_components/eh_host_nimble/src/eh_host_nimble.c`). The port is built into the `esp_hosted` component — see [Porting a BT stack to ESP-Hosted](#porting-a-bt-stack-to-esp-hosted).

### Over Hosted HCI

**Initialization** — enable the port with `CONFIG_ESP_HOSTED_HOST_FEAT_BT=y` + `CONFIG_ESP_HOSTED_HOST_BT_PORT_NIMBLE=y`. It **auto-inits at boot** (`eh_host_nimble_init()` runs once the transport is up, before `app_main` proceeds): it inits and enables the CP-side BT controller over RPC and registers the HCI RX callback (binding the TX function). The app calls **no** ESP-Hosted function — the transport TX overrides (`ble_transport_to_ll_*`) are strong link-time symbols, so NimBLE's own `ble_transport_ll_init()` is a no-op. (Bluedroid cannot auto-init — its HCI driver must attach before `esp_bluedroid_init()` — so it calls `eh_host_bluedroid_init()` explicitly; see below.)

```mermaid
sequenceDiagram
    participant app as App (example main)
    box transparent Hosted Master
    participant ble as NimBLE Host Stack
    participant hhci as Hosted HCI Bridge
    participant master as SPI/SDIO Interface
    end

    box transparent Hosted Co-processor
    participant slave as Bluetooth Controller
    end

    Note over hhci : auto-init at boot (before app_main)
    hhci ->> slave : eh_host_bt_controller_init() (RPC)
    hhci ->> slave : eh_host_bt_controller_enable() (RPC)
    hhci ->> hhci : eh_host_bt_mcu_hci_register(rx_cb) — bind TX fn

    app ->> ble : nimble_port_init()
    ble ->> +hhci : ble_transport_ll_init()
    Note over hhci : no-op — wiring already done by auto-init
    hhci -->> -ble :
```

**Sending** — ACL data is converted to HCI, a hosted header is added, and the frame crosses the bus. The co-processor removes the header and hands raw HCI to the controller.

```mermaid
sequenceDiagram
    box transparent Hosted Master
    participant ble as NimBLE Host Bluetooth Stack
    participant hhci as Hosted HCI Bridge
    participant master as SPI/SDIO Interface
    end

    box transparent Hosted Co-processor
    participant sinterface as SPI/SDIO Interface
    participant slave as Bluetooth Controller
    end

    ble ->> +hhci : ble_transport_to_ll_acl_impl()
    Note over hhci : convert ACL data to HCI
    hhci ->> +master : eh_host_transport_tx(ESP_HCI_IF)
    Note over master : add Hosted header
    master ->> +sinterface: SPI/SDIO
    Note over master,sinterface : (Hosted HCI data)
    master -->> -hhci :
    hhci -->> -ble :

    Note over sinterface : remove Hosted header
    sinterface ->> -slave : HCI data
```

**Receiving** — the controller emits HCI, the co-processor adds a hosted header, and the master removes it and dispatches into the stack as an event or as ACL data.

```mermaid
sequenceDiagram
    box transparent Hosted Master
    participant ble as NimBLE Host Bluetooth Stack
    participant hhci as Hosted HCI Bridge
    participant master as SPI/SDIO Interface
    end

    box transparent Hosted Co-processor
    participant sinterface as SPI/SDIO Interface
    participant slave as Bluetooth Controller
    end

    slave ->> +sinterface : HCI data
    Note over sinterface : Add Hosted header
    sinterface ->> -master : SPI/SDIO
    Note over sinterface,master : (Hosted HCI data)
    Note over master : Remove Hosted header

    master ->> +hhci : eh_host_mcu_hci_rx_handler_fn()

    alt Receive Event Data
        Note over hhci: convert HCI data to Event
        hhci ->> ble : ble_transport_to_hs_evt()
        ble -->> hhci :
    else Receive ACL Data
        Note over hhci: convert HCI data to ACL
        hhci ->> ble : ble_transport_to_hs_acl()
        ble -->> hhci :
    end

    hhci -->> -master :
```

### Over standard UART HCI

With standard HCI the driver is UART, talking directly to a controller that exposes a UART interface — no hosted header, no multiplexing.

**Initialization** — the app opens the UART link with `hci_uart_open()` (configures the port and starts the RX task); NimBLE's `ble_transport_ll_init()` is then a no-op. Teardown closes the port via `ble_transport_ll_deinit()` → `hci_uart_close()`.

```mermaid
sequenceDiagram
    participant app as App (example main)
    box transparent Master
    participant ble as NimBLE Host Stack
    participant huart as UART Driver
    end

    box transparent Co-processor
    participant slave as Bluetooth Controller with UART Interface
    end

    app ->> huart : hci_uart_open() (configure UART + start RX task)
    huart -->> app : ready
    app ->> ble : nimble_port_init()
    ble ->> huart : ble_transport_ll_init()
    Note over huart : no-op — UART opened by hci_uart_open()
    huart -->> ble :
```

**Sending:**

```mermaid
sequenceDiagram
    box transparent Master
    participant ble as NimBLE Host Bluetooth Stack
    participant huart as UART Driver
    end

    box transparent Co-processor
    participant slave as Bluetooth Controller with UART Interface
    end

    ble ->> huart : ble_transport_to_ll_acl_impl()
    Note over huart : convert ACL data to HCI
    huart ->> slave : UART TX
    Note over huart,slave : (standard HCI)
    huart -->> ble :
```

**Receiving:**

```mermaid
sequenceDiagram
    box transparent Master
    participant ble as NimBLE Host Bluetooth Stack
    participant huart as UART Driver
    end

    box transparent Co-processor
    participant slave as Bluetooth Controller with UART Interface
    end

    slave ->> huart : UART RX
    Note over slave,huart: (standard HCI)

    alt Receive Event Data
        Note over huart : convert HCI data to Event
        huart ->> ble : ble_transport_to_hs_evt()
        ble -->> huart :
    else Receive ACL Data
        Note over huart : convert HCI data to ACL
        huart ->> ble : ble_transport_to_hs_acl()
        ble -->> huart :
    end
```

---

## BlueDroid host stack

Hosted implements the BlueDroid transport API set: `hosted_hci_bluedroid_open`, `hosted_hci_bluedroid_close`, `hosted_hci_bluedroid_send`, `hosted_hci_bluedroid_check_send_available`, `hosted_hci_bluedroid_register_host_callback`.

`hosted_hci_bluedroid_open` must be called by the application before attaching the transport APIs to BlueDroid and starting it — this initialises the underlying transport. `hosted_hci_bluedroid_register_host_callback` records BlueDroid's callback (`notify_host_recv`) used to deliver incoming HCI data to the stack.

### Over Hosted HCI

**Initialization** — enable the port with `CONFIG_ESP_HOSTED_HOST_FEAT_BT=y` + `CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID=y`. Bluedroid's HCI driver must attach **before** `esp_bluedroid_init()`, an ordering the async auto-init task can't guarantee, so set `CONFIG_ESP_HOSTED_HOST_BT_PORT_BLUEDROID_AUTO_INIT=n` and call the port helper `eh_host_bluedroid_init()` explicitly in `app_main` (it inits + enables the controller and attaches the HCI driver). A bring-your-own-driver app can instead wire the `hosted_hci_bluedroid_*` ops into `esp_bluedroid_hci_driver_operations_t` itself (see `examples/bluetooth/esp_hosted_bluedroid/controller_mac_addr`):

```mermaid
sequenceDiagram
    box transparent Hosted Master
    participant bt as Host Application
    participant hhci as Hosted HCI Bridge
    participant master as SPI/SDIO Interface
    end

    box transparent Hosted Co-processor
    participant sinterface as SPI/SDIO Interface
    participant slave as Bluetooth Controller
    end

    bt ->> +hhci : hosted_hci_bluedroid_open()
    Note over hhci: do any init required
    hhci -->> -bt :
```

**Sending:**

```mermaid
sequenceDiagram
    box transparent Hosted Master
    participant bt as BlueDroid Host Bluetooth Stack
    participant hhci as Hosted HCI Bridge
    participant master as SPI/SDIO Interface
    end

    box transparent Hosted Co-processor
    participant sinterface as SPI/SDIO Interface
    participant slave as Bluetooth Controller
    end

    bt ->> +hhci : hosted_hci_bluedroid_send()
    Note over hhci : HCI data
    hhci ->> +master : eh_host_transport_tx(ESP_HCI_IF)
    Note over master : add Hosted header
    master ->> +sinterface: SPI/SDIO
    Note over master,sinterface : (Hosted HCI data)
    master -->> -hhci :
    hhci -->> -bt :

    Note over sinterface : remove Hosted header
    sinterface ->> -slave : HCI data
```

**Receiving:**

```mermaid
sequenceDiagram
    box transparent Hosted Master
    participant bt as BlueDroid Host Bluetooth Stack
    participant hhci as Hosted HCI Bridge
    participant master as SPI/SDIO Interface
    end

    box transparent Hosted Co-processor
    participant sinterface as SPI/SDIO Interface
    participant slave as Bluetooth Controller
    end

    slave ->> +sinterface : HCI data
    Note over sinterface : Add Hosted header
    sinterface ->> -master : SPI/SDIO
    Note over sinterface,master : (Hosted HCI data)
    Note over master : Remove Hosted header

    master ->> +hhci : eh_host_mcu_hci_rx_handler_fn()

    hhci ->> bt : notify_host_recv()
    Note over hhci, bt: HCI data

    hhci -->> -master :
```

### Over standard UART HCI

Using BlueDroid over UART needs UART functions that:

- `uart_open` — open and initialise the UART driver (GPIOs, baud rate, etc.).
- `uart_tx` — transmit over UART.
- `UART RX` — a thread waiting for incoming UART data.
- `notify_host_recv` — a BlueDroid callback registered with `UART RX` to receive UART data.

`uart_open` is called before starting BlueDroid; `uart_tx` and `notify_host_recv` are registered by BlueDroid with the UART driver.

**Initialization:**

```mermaid
sequenceDiagram
    box transparent Master
    participant bt as Host Application
    participant huart as UART Driver
    end

    box transparent Co-processor
    participant slave as Bluetooth Controller with UART Interface
    end

    bt ->> +huart : uart_open()
    Note over huart: do any uart init required
    huart -->> -bt :
```

**Sending:**

```mermaid
sequenceDiagram
    box transparent Master
    participant bt as BlueDroid Host Bluetooth Stack
    participant huart as UART Driver
    end

    box transparent Co-processor
    participant slave as Bluetooth Controller with UART Interface
    end

    bt ->> huart : uart_tx()
    huart ->> slave : UART TX
    Note over huart,slave : (standard HCI)
    huart -->> bt :
```

**Receiving:**

```mermaid
sequenceDiagram
    box transparent Master
    participant bt as BlueDroid Host Bluetooth Stack
    participant huart as UART Driver
    end

    box transparent Co-processor
    participant slave as Bluetooth Controller with UART Interface
    end

    slave ->> huart : UART RX
    Note over slave,huart: (standard HCI)
    huart ->> bt : notify_host_recv()
    Note over huart, bt: HCI data

    bt -->> huart:
```

---

## Implementation notes: standard HCI over UART

Standard HCI over UART is configured through the Bluetooth component Kconfig. In `menuconfig`, go to `Component config` → `Bluetooth` → `Controller Options` → `HCI mode` (or `HCI Config`) and set it to `UART(H4)`.

```text
Component config
└── Bluetooth
    └── Controller Options
        └── HCI mode / HCI Config ──> UART(H4)
```

Depending on the co-processor, UART parameters (Tx/Rx pins, hardware flow control, RTS/CTS pins, baud rate) are configured through the Bluetooth component. Any UART parameters the Bluetooth component does not handle are set by ESP-Hosted under `Example Configuration` → `HCI UART Settings`.

> [!NOTE]
> Make sure the Standard-HCI UART GPIO pins do not conflict with the GPIO pins used by the selected ESP-Hosted transport.

---

## Porting a BT stack to ESP-Hosted

ESP-Hosted is stack-agnostic. The `eh_host_feat_bt` feature provides a raw HCI
byte-pipe to the co-processor controller; a **stack port** is the thin glue that
binds a particular host stack to that pipe. The two reference ports —
`examples/common_components/eh_host_nimble` and `.../eh_host_bluedroid` — are
built into the `esp_hosted` component (gated by Kconfig) and are the template
for your own.

**What `feat_bt` gives the port** (`eh_host_feat_bt.h`, `eh_host_feat_bt_mcu.h`):

- `eh_host_bt_controller_init/enable/disable/deinit()` — CP-side controller
  lifecycle over RPC (tolerate `ESP_ERR_INVALID_STATE`: `feat_bt` may already
  have brought the controller up).
- `eh_host_bt_mcu_hci_register(rx_cb, user)` → returns the **TX function** for
  HCI frames (H4: `pkt-type | body`); `eh_host_bt_mcu_hci_unregister()`.

**What the port does** — implement two directions and a lifecycle:

1. **TX** — convert your stack's outgoing HCI to an H4 frame and hand it to the
   TX function returned by `eh_host_bt_mcu_hci_register`.
2. **RX** — in the `rx_cb`, parse the inbound H4 frame and dispatch it into your
   stack (event vs ACL).
3. **`eh_host_<stack>_init()` / `_deinit()`** — public `esp_err_t` entry points:
   init = controller init/enable + `eh_host_bt_mcu_hci_register` (bind RX, keep
   TX); deinit = unregister + controller disable/deinit. Do **not** call
   `connect_to_slave` — transport bring-up is automatic.

**Wire it in:**

- **Kconfig** (`Kconfig.ext`): `config ESP_HOSTED_HOST_BT_PORT_<STACK>`
  (depends on `ESP_HOSTED_HOST_FEAT_BT` and your stack being enabled) plus
  `..._AUTO_INIT` (`default y`, `depends on ESP_HOSTED_HOST_AUTO_FEAT_INIT`).
- **Auto-init** (`eh_host_auto_init.h`): under `#if CONFIG_..._AUTO_INIT`,
  `EH_HOST_FEAT_REGISTER(eh_host_<stack>_init, eh_host_<stack>_deinit,
  "bt_port_<stack>", 250)` — priority 250 runs after `feat_bt` (150) so the HCI
  byte-pipe is up first.
- **CMake**: `add_library(eh_host_<stack> STATIC ...)` gated by
  `if(NOT CONFIG_ESP_HOSTED_HOST_BT_PORT_<STACK>) return()`; link
  `eh_host_feat_bt` (+ `eh_host_autoinit` when auto-init is on) and your stack;
  `target_link_options(... INTERFACE "-Wl,--undefined=eh_host_<stack>_init")` to
  force the TU into the link; self-attach with
  `target_link_libraries(eh_host INTERFACE eh_host_<stack>)`.

**Two binding patterns** (from the reference ports):

- **Link-time override** (NimBLE): the stack declares weak transport hooks
  (`ble_transport_to_ll_*`); the port provides strong overrides. No app call —
  it can **auto-init** at boot.
- **Driver-ops attach** (Bluedroid): the app attaches an HCI driver ops struct
  (`hosted_hci_bluedroid_*`) before initialising the stack. Because the attach
  must complete **synchronously before** the stack's init, such a port sets
  `..._AUTO_INIT=n` and the example calls `eh_host_<stack>_init()` explicitly.

See the working ports and examples: `examples/bluetooth/esp_hosted_nimble/` and
`examples/bluetooth/esp_hosted_bluedroid/`.

---

## References

- [esp-nimble](https://github.com/espressif/esp-nimble)
- [ESP-IDF NimBLE host APIs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/nimble/index.html)
- [ESP-IDF Bluetooth API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/index.html)
- [NimBLE host-only over UART example](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/nimble/bleprph_host_only)
- [BlueDroid host-only over UART example](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/bluedroid_host_only/bluedroid_host_only_uart)
