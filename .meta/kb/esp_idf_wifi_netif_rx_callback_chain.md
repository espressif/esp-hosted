# ESP-IDF WiFi Netif RX Callback Chain

How WiFi received packets reach a netif in ESP-IDF v5.5.
Source files examined: `esp_wifi/src/wifi_netif.c`, `esp_wifi/src/wifi_default.c`,
`esp_wifi/include/esp_wifi_netif.h`.

---

## Data Structures

```
wifi_netif.c (internal to esp_wifi component):

    s_wifi_rxcbs[MAX_WIFI_IFS]   — per-interface esp_netif_receive_t callback
    s_wifi_netifs[MAX_WIFI_IFS]  — per-interface esp_netif_t pointer

wifi_default.c (internal to esp_wifi component):

    s_wifi_netifs[MAX_WIFI_IFS]  — SEPARATE array, same name, same purpose
                                   populated by esp_wifi_set_default_wifi_sta_handlers()
```

## The Two-Level Callback Architecture

```
WiFi driver (binary blob, libnet80211.a)
    │
    │ calls: wlan_rxcb registered via esp_wifi_internal_reg_rxcb()
    │
    ▼
wifi_sta_receive()                          [wifi_netif.c:37]
    │
    │ calls: s_wifi_rxcbs[WIFI_IF_STA](s_wifi_netifs[WIFI_IF_STA], buffer, len, eb)
    │
    ▼
esp_netif_receive(netif, buffer, len, eb)   [registered via esp_wifi_register_if_rxcb]
    │
    │ calls: netif->lwip_input_fn (= wlanif_input for WiFi STA)
    │
    ▼
wlanif_input(netif_handle, buffer, len, eb) [lwip wlanif.c]
    │
    │ creates pbuf, then calls: lwip_netif->input(pbuf, lwip_netif)
    │ where input was set by netif_add() to tcpip_input
    │
    ▼
tcpip_input(pbuf, netif)                    [lwip tcpip.c — posts to tcpip thread mbox]
```

### Level 1: `esp_wifi_internal_reg_rxcb()`

- Registers a `wifi_rxcb_t` callback with the WiFi driver binary blob.
- Signature: `esp_err_t (*wifi_rxcb_t)(void *buffer, uint16_t len, void *eb)`
- Only ONE callback per interface (STA/AP/NAN). Last writer wins.
- The WiFi driver calls this for every received 802.11 data frame.

### Level 2: `esp_wifi_register_if_rxcb()`

- Higher-level API in `wifi_netif.c`.
- Stores the actual handler in `s_wifi_rxcbs[]` and the netif in `s_wifi_netifs[]`.
- Then registers `wifi_sta_receive` (a thin wrapper) as the Level-1 callback.
- `wifi_sta_receive` calls `s_wifi_rxcbs[STA](s_wifi_netifs[STA], buf, len, eb)`.

```c
// wifi_netif.c:37
static esp_err_t wifi_sta_receive(void *buffer, uint16_t len, void *eb)
{
    return s_wifi_rxcbs[WIFI_IF_STA](s_wifi_netifs[WIFI_IF_STA], buffer, len, eb);
}

// wifi_netif.c:135
esp_err_t esp_wifi_register_if_rxcb(wifi_netif_driver_t ifx, esp_netif_receive_t fn, void *arg)
{
    // ...
    s_wifi_rxcbs[wifi_interface] = fn;       // store actual handler
    // ...
    s_wifi_netifs[wifi_interface] = ifx->base.netif;  // store netif
    esp_wifi_internal_reg_rxcb(wifi_interface, rxcb);  // register wrapper
    // where rxcb = wifi_sta_receive for STA
}
```

## `esp_wifi_set_default_wifi_sta_handlers()` Internals

Defined in `wifi_default.c`. Registers event handlers for the WiFi STA netif lifecycle.
Called with the netif obtained from `esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")`.

### STA_START handler: `wifi_default_action_sta_start` → `wifi_start()`

```c
// wifi_default.c:43
static void wifi_start(void *esp_netif, ...) {
    wifi_netif_driver_t driver = esp_netif_get_io_driver(esp_netif);
    // ...
    if (esp_wifi_is_if_ready_when_started(driver)) {      // FALSE for STA
        esp_wifi_register_if_rxcb(driver, esp_netif_receive, esp_netif);
    }
    // ...
    esp_netif_action_start(esp_netif, ...);    // → netif_add() → sets netif->input
}
```

- `esp_wifi_is_if_ready_when_started()` returns FALSE for STA (only true for AP).
- So on STA_START: `esp_wifi_register_if_rxcb` is NOT called.
- `esp_netif_action_start` IS called → `netif_add()` → `netif->input = tcpip_input`.

### STA_CONNECTED handler: `wifi_default_action_sta_connected`

```c
// wifi_default.c:97
static void wifi_default_action_sta_connected(...) {
    // ...
    wifi_netif_driver_t driver = esp_netif_get_io_driver(esp_netif);

    if (!esp_wifi_is_if_ready_when_started(driver)) {     // TRUE for STA
        // rxcb registered on connection, not on start
        esp_wifi_register_if_rxcb(driver, esp_netif_receive, esp_netif);
    }

    esp_netif_action_connected(esp_netif, ...);           // → netif_set_link_up()
}
```

- For STA: `!esp_wifi_is_if_ready_when_started()` is TRUE.
- So on EVERY STA_CONNECTED, `esp_wifi_register_if_rxcb()` is called.
- This calls `esp_wifi_internal_reg_rxcb(STA, wifi_sta_receive)`.
- **This OVERWRITES whatever Level-1 callback was previously registered.**

### STA_DISCONNECTED handler: `wifi_default_action_sta_disconnected`

```c
// wifi_default.c:126
static void wifi_default_action_sta_disconnected(...) {
    // ...
    esp_netif_action_disconnected(esp_netif, ...);        // → netif_set_link_down()
}
```

- Does NOT touch the RX callback.
- Only calls `netif_set_link_down()` on the lwIP netif.

### STA_STOP handler: `wifi_default_action_sta_stop`

```c
// wifi_default.c:84
static void wifi_default_action_sta_stop(...) {
    esp_netif_action_stop(esp_netif, ...);    // → netif_remove() → clears netif->input
}
```

### Summary: When RX callback changes and when netif->input changes

| Event | RX callback (Level-1) | netif->input |
|-------|----------------------|--------------|
| STA_START | NOT changed (STA skips rxcb registration) | SET by `netif_add()` → `tcpip_input` |
| STA_CONNECTED | **OVERWRITTEN** to `wifi_sta_receive` by `esp_wifi_register_if_rxcb()` | Not changed |
| STA_DISCONNECTED | Not changed | Not changed (`netif_set_link_down` only) |
| STA_STOP | Not changed | **CLEARED** by `netif_remove()` → NULL |

## `esp_netif_attach_wifi_station()` — What It Does

```c
// esp_netif_lwip.c or esp_wifi component
esp_err_t esp_netif_attach_wifi_station(esp_netif_t *esp_netif) {
    wifi_netif_driver_t driver = esp_wifi_create_if_driver(WIFI_IF_STA);
    return esp_netif_attach(esp_netif, driver);
}
```

- Creates a WiFi netif driver (`wifi_netif_driver_t`).
- `esp_netif_attach()` calls `driver->base.post_attach` = `wifi_driver_start()`.
- `wifi_driver_start()` sets TX config (`esp_wifi_internal_tx`, `wifi_free`).
- Does NOT register RX callback. RX is registered later by the default handlers.

## `esp_wifi_is_if_ready_when_started()` — The STA vs AP Difference

```c
// wifi_netif.c:125
bool esp_wifi_is_if_ready_when_started(wifi_netif_driver_t ifx)
{
#ifdef CONFIG_ESP_WIFI_SOFTAP_SUPPORT
    // WiFi rxcb to be registered on start for AP only,
    // station gets it registered on connect event
    return (ifx && ifx->wifi_if == WIFI_IF_AP);
#else
    return false;
#endif
}
```

- AP: rxcb registered on STA_START (AP is "ready when started").
- STA: rxcb registered on STA_CONNECTED (STA is NOT "ready when started").
- This is by design: STA needs to connect to an AP before receiving data frames.

---

## Remark: ESP-Hosted nw_split crash — root cause

### Setup

The ESP-Hosted coprocessor manages the WiFi RX callback via its own API:
`eh_cp_rx_register(ESP_STA_IF, callback)`, which internally calls
`esp_wifi_internal_reg_rxcb(WIFI_IF_STA, callback)`.

At init, the core registers `default_wlan_sta_rx_callback` (sends packets to host
transport). The nw_split component later overrides this with a filtering callback
that routes packets to either the host or the CP's local lwIP netif.

The nw_split component also calls `esp_wifi_set_default_wifi_sta_handlers()` to
register ESP-IDF's default netif lifecycle handlers for the CP's local netif. These
handlers manage `netif_add`/`netif_remove`/`netif_set_link_up`/`netif_set_link_down`.

### The conflict

On every STA_CONNECTED event, the ESP-IDF default handler
(`wifi_default_action_sta_connected`) calls `esp_wifi_register_if_rxcb()`, which
calls `esp_wifi_internal_reg_rxcb(STA, wifi_sta_receive)`. This **overwrites** the
nw_split filtering callback with `wifi_sta_receive`.

`wifi_sta_receive` calls `esp_netif_receive(netif, buffer, len, eb)` directly —
with no guards for `netif->input` being NULL.

The nw_split STA_CONNECTED handler runs AFTER the default handler (registered later)
and re-overrides the callback back to the nw_split filter. But there is a window
between the two handlers where `wifi_sta_receive` is active.

### Why it crashes

When `CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START=y` is set
(only in the `network_split__host_power_save` example), the nw_split STA_START handler
calls `eh_cp_feat_wifi_request_connect()`. This starts WiFi connection.

Simultaneously, the example's own STA_START handler and/or the host via RPC also
call `esp_wifi_connect()`. This causes a connect conflict → disconnect reason 36 →
auto-reconnect.

During reconnection, the following happens on STA_CONNECTED:

```
Event handler execution order (determined by registration order):

1. Default handler (registered first, by esp_wifi_set_default_wifi_sta_handlers):
   → esp_wifi_register_if_rxcb(driver, esp_netif_receive, netif)
     → esp_wifi_internal_reg_rxcb(STA, wifi_sta_receive)
     → OVERWRITES the nw_split callback

2. WiFi packets arrive (WiFi driver task, higher priority than event loop):
   → wifi_sta_receive(buffer, len, eb)
     → s_wifi_rxcbs[STA](s_wifi_netifs[STA], buffer, len, eb)
     → esp_netif_receive(netif, buffer, len, eb)
       → wlanif_input(netif, buffer, len, eb)
         → lwip_netif->input(pbuf, lwip_netif)
         → netif->input is NULL → CRASH

   netif->input can be NULL because:
   - The disconnect/reconnect cycle may have left the netif in an inconsistent state
   - Or netif_add ran but netif_remove was called during the reconnect cycle
   - Or the netif was never fully initialized for this reconnect cycle

3. nw_split handler (registered second):
   → nw_split_try_override_sta_rx_cb()
     → eh_cp_rx_register(STA, nw_split_filter_cb)
   → Too late — crash already happened at step 2
```

### Why `nw_split` test passes but `nw_split_ps` crashes

| Config | `network_split/station` | `network_split__host_power_save` |
|--------|------------------------|----------------------------------|
| `NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START` | **Not set** | **Set (=y)** |
| `FEAT_HOST_PS` | Not set | Set |
| `FEAT_CLI` | Not set | Set |
| `EVENT_TASK_STACK_SIZE` | Default | 4096 |

Without `AUTO_CONNECT_ON_STA_START`, the connection sequence is cleaner — no
double-connect conflict, no reason-36 disconnect, no reconnect race. The default
handler's rxcb override on STA_CONNECTED still happens, but the nw_split handler
immediately re-overrides, and by that time `netif->input` is valid (no prior
disconnect corrupted the state).

With `AUTO_CONNECT_ON_STA_START=y`, the auto-connect from nw_split's STA_START
handler races with the example's STA_START handler and/or host RPC. This causes
a disconnect/reconnect cycle during which the rxcb override window is hit with
an invalid netif.

### Fix direction

Do NOT use `esp_wifi_set_default_wifi_sta_handlers()`. Instead, register custom
handlers that perform `esp_netif_action_start`, `esp_netif_action_connected`,
etc. WITHOUT calling `esp_wifi_register_if_rxcb()`. The ESP-Hosted RX callback
chain (`eh_cp_rx_register`) is the sole authority for who receives WiFi packets.
