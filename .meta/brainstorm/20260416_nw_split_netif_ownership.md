<!-- %% me.br-260416nso.o %% - always -->
# Brainstorm: Network Split — Wrap-Layer WiFi State Management

**Date**: 2026-04-16  |  **Status**: Active
**Trigger**: `nw_split_ps` crash — NULL `netif->input` in `wlanif_input:177`

Color key for all diagrams:
- 🟩 `#c8e6c9` = safe / hosted core
- 🟦 `#bbdefb` = WiFi feature (wrap owner)
- 🟧 `#ffe0b2` = nw_split
- 🟥 `#ffcdd2` = error / conflict
- ⬜ `#e0e0e0` = passthrough / no-op

---

## 1. The Problem: 5 Actors, No Coordinator

Five actors modify WiFi RX callback and WiFi state without coordination.

```mermaid
graph TD
    subgraph "WiFi RX Callback — 5 uncoordinated writers"
        CB["esp_wifi_internal_reg_rxcb(STA)"]
    end

    A1(["eh_cp_core<br/>init: sets default_wlan_sta_rx_cb"])
    A2(["eh_cp_feat_wifi<br/>DISCONNECT: NULLs it ⚠️<br/>CONNECT: re-registers<br/>STOP: NULLs it ⚠️"])
    A3(["eh_cp_feat_nw_split<br/>CONNECT: overrides to filter cb<br/>DISCONNECT: restores orig"])
    A4(["App example<br/>calls esp_wifi_connect etc."])
    A5(["Host via RPC<br/>calls esp_wifi_connect etc."])

    A1 -->|"eh_cp_rx_register()"| CB
    A2 -->|"DIRECT call bypasses core"| CB
    A3 -->|"eh_cp_rx_register()"| CB
    A4 -.->|"goes through wraps"| CB
    A5 -.->|"goes through wraps"| CB

    style A1 fill:#c8e6c9,color:#000
    style A2 fill:#ffcdd2,color:#000
    style A3 fill:#ffe0b2,color:#000
    style A4 fill:#e0e0e0,color:#000
    style A5 fill:#e0e0e0,color:#000
    style CB fill:#ffcdd2,color:#000
```

**The crash sequence:**

```mermaid
sequenceDiagram
    participant WF as WiFi Feature
    participant NWS as nw_split
    participant CB as RX Callback

    Note over CB: cb = nw_split_filter_cb ✓

    Note over WF,CB: WiFi disconnects
    WF->>CB: reg_rxcb(STA, NULL) ← direct, bypasses core
    Note over CB: cb = NULL ⚠️
    NWS->>CB: eh_cp_rx_register(orig_host_cb)
    Note over CB: cb = orig_host_cb

    Note over WF,CB: WiFi reconnects
    WF->>CB: reg_rxcb(STA, eh_cp_rx_get()) ← reads stale s_rx_cb
    Note over CB: cb = orig_host_cb (not nw_split!)
    Note over NWS: Packets arrive → go to host, skip nw_split
    NWS->>CB: eh_cp_rx_register(nw_split_cb) ← late
    Note over NWS: But between WF and NWS handlers,<br/>netif->input may be NULL → crash
```

---

## 2. The Solution: Wrap Layer in `eh_cp_feat_wifi`

### Why `eh_cp_feat_wifi`, not `eh_cp_core`

```
eh_cp_core          — transport, handshake, s_rx_cb[], eh_cp_rx_register()
  └─ eh_cp_feat_wifi  — WiFi authority: state, wraps, event publishing
       └─ eh_cp_feat_nw_split  — packet routing (sits ON TOP of WiFi)
            └─ eh_cp_feat_host_ps  — sleep flag
```

`eh_cp_feat_wifi` already owns:
- `__wrap_esp_wifi_connect()` + dedup logic
- `s_station_connecting`, `s_wifi_started` state
- `s_pending_sta_config` caching
- All STA/AP event handlers
- The direct `esp_wifi_internal_reg_rxcb` calls (which need fixing)

Moving ALL wraps here consolidates WiFi authority in one place.
Core stays lean: transport + handshake + callback array.


### State Machine

```mermaid
stateDiagram-v2
    [*] --> W_NONE

    W_NONE --> W_INIT: wrap_wifi_init
    W_INIT --> W_STARTED: wrap_wifi_start
    W_STARTED --> W_CONNECTING: wrap_wifi_connect
    W_CONNECTING --> W_CONNECTED: STA_CONNECTED evt
    W_CONNECTING --> W_STARTED: STA_DISCONNECTED evt
    W_CONNECTED --> W_STARTED: STA_DISCONNECTED evt
    W_CONNECTED --> W_DISCONNECTING: wrap_wifi_disconnect
    W_DISCONNECTING --> W_STARTED: STA_DISCONNECTED evt
    W_STARTED --> W_INIT: wrap_wifi_stop
    W_INIT --> W_NONE: wrap_wifi_deinit

    state "Netif (tracked in parallel)" as NS {
        N_NONE --> N_READY: STA_START evt (netif_add → input set)
        N_READY --> N_NONE: STA_STOP evt (netif_remove → input NULL)
    }
```

### What Each Wrap Does

```mermaid
flowchart LR
    subgraph wraps["All wraps in eh_cp_feat_wifi"]
        direction TB
        W1["<b>wrap_wifi_init</b><br/>same config → no-op<br/>changed + nw_split → suspend/reinit<br/>track: wifi=INIT"]
        W2["<b>wrap_wifi_start</b><br/>already started → no-op<br/>track: wifi=STARTED"]
        W3["<b>wrap_wifi_stop</b><br/>nw_split active → refuse<br/>track: wifi=INIT"]
        W4["<b>wrap_wifi_connect</b><br/>already connecting → dedup<br/>track: wifi=CONNECTING"]
        W5["<b>wrap_wifi_set_config</b><br/>wifi connecting → cache config<br/>apply after connect resolves"]
        W6["<b>wrap_wifi_disconnect</b><br/>track: wifi=DISCONNECTING"]
        W7["<b>wrap_wifi_deinit</b><br/>nw_split active → refuse<br/>track: wifi=NONE"]
    end

    style wraps fill:#bbdefb,color:#000
```

---

## 3. Auto-Detection: Zero Kconfig for Ownership

nw_split detects the system state at init and adapts. No ownership Kconfig needed.

```mermaid
flowchart TD
    NWS["nw_split init"] --> Q1{"WIFI_STA_DEF<br/>netif exists?"}

    Q1 -->|Yes| REUSE["Reuse app's netif<br/>Skip create<br/>Skip default handlers"]
    Q1 -->|No| CREATE["Create netif<br/>Register default handlers"]

    REUSE --> INSTALL["Install permanent RX callback<br/>nw_split_active = true"]
    CREATE --> INSTALL

    INSTALL --> Q2{"WiFi already<br/>started?"}
    Q2 -->|No| WAIT["Wait for STA_START<br/>(permanent cb in passthrough)"]
    Q2 -->|Yes| Q3{"Already<br/>connected?"}
    Q3 -->|No| WAIT2["Wait for STA_CONNECTED<br/>(permanent cb in passthrough)"]
    Q3 -->|Yes| ACTIVE["Start filtering immediately"]

    style REUSE fill:#ffe0b2,color:#000
    style CREATE fill:#ffe0b2,color:#000
    style INSTALL fill:#ffe0b2,color:#000
    style WAIT fill:#e0e0e0,color:#000
    style WAIT2 fill:#e0e0e0,color:#000
    style ACTIVE fill:#c8e6c9,color:#000
```

**All valid app patterns — zero config required:**

| Pattern | App code | nw_split detects | Result |
|---------|----------|-----------------|--------|
| A | `eh_cp_init()` only, host drives WiFi via RPC | No netif, no WiFi | Creates netif, waits for host |
| B | `eh_cp_init()` → `wifi_init` → `wifi_start` → `connect` | No netif, no WiFi at init | Creates netif, app drives WiFi |
| C | `create_netif()` → `eh_cp_init()` → `wifi_start` | Netif exists | Reuses netif, app drives WiFi |
| D | `create_netif` → `wifi_start` → `connect` → `eh_cp_init()` | Everything running | Reuses netif, starts filtering immediately |

---

## 4. Permanent RX Callback

Installed once at nw_split init. Never swapped. All routing in one place.

```mermaid
flowchart TD
    PKT["WiFi RX packet"] --> Q1{"sta_netif<br/>exists?"}
    Q1 -->|No| PASS1["→ host<br/>(passthrough)"]

    Q1 -->|Yes| Q2{"netif STARTED<br/>+ input valid?"}
    Q2 -->|No| PASS2["→ host<br/>(netif not ready)"]

    Q2 -->|Yes| Q3{"host<br/>sleeping?"}
    Q3 -->|Yes| FILTER_PS["Filter:<br/>HOST → drop<br/>CP → deliver<br/>BOTH → CP only"]
    Q3 -->|No| FILTER["Filter:<br/>HOST → host transport<br/>CP → esp_netif_receive<br/>BOTH → copy + both"]

    Q2 -->|"input==NULL<br/>(should never happen)"| ASSERT["ASSERT + log:<br/>'netif destroyed while<br/>nw_split active'"]

    style PASS1 fill:#e0e0e0,color:#000
    style PASS2 fill:#e0e0e0,color:#000
    style FILTER fill:#c8e6c9,color:#000
    style FILTER_PS fill:#ffe0b2,color:#000
    style ASSERT fill:#ffcdd2,color:#000
```

**What this eliminates:**

| Removed | Was |
|---------|-----|
| `s_sta_rx_overridden` flag | Tracked override state |
| `s_sta_netif_input_ready` flag | Racy cached readiness |
| `nw_split_try_override_sta_rx_cb()` | 18 lines, called from 2 events |
| `eh_cp_feat_nw_split_reset_rx_override()` | Full state reset |
| `nw_split_get_active_sta_netif()` runtime lookup | Key-based hunt every packet |
| STA_CONNECTED/DISCONNECTED callback swap | Override/restore dance |

---

## 5. Not Presumptuous — What the System Does vs Doesn't

```mermaid
graph LR
    subgraph does["System DOES (safe, implicit)"]
        D1["Create netif<br/><i>nw_split needs it</i>"]
        D2["Install permanent cb<br/><i>that IS nw_split</i>"]
        D3["Dedup connect calls<br/><i>prevents driver error</i>"]
        D4["Cache set_config<br/><i>apply after busy</i>"]
        D5["Refuse wifi_stop<br/><i>prevents crash</i>"]
    end

    subgraph doesnt["System DOES NOT (user decides)"]
        N1["wifi_init"]
        N2["wifi_start"]
        N3["wifi_connect"]
        N4["set SSID/password"]
        N5["auto-reconnect"]
    end

    style does fill:#c8e6c9,color:#000
    style doesnt fill:#ffcdd2,color:#000
```

---

## 6. Code Changes

### File ownership after changes

```mermaid
graph TD
    subgraph core["eh_cp_core"]
        C1["s_rx_cb[] array"]
        C2["eh_cp_rx_register()"]
        C3["default_wlan_sta_rx_callback"]
    end

    subgraph wifi_feat["eh_cp_feat_wifi (WiFi authority)"]
        W1["WiFi state machine"]
        W2["All __wrap_esp_wifi_*"]
        W3["Event publisher"]
        W4["Connect/disconnect helpers"]
        W5["Config caching"]
        W_NOTE["Does NOT call<br/>esp_wifi_internal_reg_rxcb<br/>directly anymore"]
    end

    subgraph nw_split["eh_cp_feat_nw_split"]
        NS1["Permanent RX callback<br/>(installed once via core)"]
        NS2["Netif create/reuse<br/>(auto-detected)"]
        NS3["Packet filter rules"]
        NS_NOTE["Does NOT call<br/>esp_wifi_* APIs<br/>Does NOT swap callbacks"]
    end

    subgraph rpc["eh_cp_feat_rpc_ext_*"]
        R1["Translates host RPC → wraps"]
        R_NOTE["Does NOT own<br/>any WiFi state"]
    end

    core --> wifi_feat --> nw_split
    rpc -->|"calls"| wifi_feat

    style core fill:#c8e6c9,color:#000
    style wifi_feat fill:#bbdefb,color:#000
    style nw_split fill:#ffe0b2,color:#000
    style rpc fill:#e0e0e0,color:#000
    style W_NOTE fill:#fff9c4,color:#000
    style NS_NOTE fill:#fff9c4,color:#000
    style R_NOTE fill:#fff9c4,color:#000
```

### All changes

| File | Change | Why |
|------|--------|-----|
| **`eh_cp_feat_wifi/CMakeLists.txt`** | Add all `--wrap=esp_wifi_*` linker flags | Centralize wraps in WiFi authority |
| **`eh_cp_feat_wifi/src/eh_cp_feat_wifi_state.c`** | **NEW**: state struct + all wrap implementations | Single WiFi state machine |
| **`eh_cp_feat_wifi/include/eh_cp_feat_wifi_state.h`** | **NEW**: query API (`is_started`, `is_connecting`, `netif_is_ready`, etc.) | Features read state |
| **`eh_cp_feat_wifi/src/eh_cp_feat_wifi_event_publisher.c`** | **REMOVE** 3 direct `esp_wifi_internal_reg_rxcb` calls (lines 265, 332, 350) | Stop bypassing core. Permanent cb handles it. |
| **`eh_cp_feat_rpc_ext_mcu/src/.._handler_req_wifi.c`** | **MOVE** `__wrap_esp_wifi_init` to WiFi feature | Was in wrong component |
| **`eh_cp_feat_rpc_ext_mcu/CMakeLists.txt`** | **REMOVE** `--wrap=esp_wifi_init` | Moved to WiFi feature |
| **`eh_cp_feat_nw_split/src/eh_cp_feat_nw_split.c`** | Remove override dance (~80 lines). Install permanent cb once at init. Simplify event handlers. | Permanent callback design |
| **`eh_cp_core/src/eh_cp_core.c`** | No structural changes. `eh_cp_rx_register()` stays here. | Core keeps callback array |

### Phased implementation

| Phase | Scope | What it fixes |
|-------|-------|---------------|
| **P1** | Remove 3 direct `reg_rxcb` calls from WiFi feature | Fixes the 5th-actor bypass — may fix the crash alone |
| **P2** | Move `__wrap_esp_wifi_init` to WiFi feature, add `__wrap_esp_wifi_stop` guard | Prevents netif destruction during nw_split |
| **P3** | Permanent RX callback in nw_split (remove override dance) | Eliminates all callback-swap TOCTOU races |
| **P4** | Add `__wrap_esp_wifi_set_config` caching, `__wrap_esp_wifi_connect` dedup (extend existing) | Handles 4-actor connect contention gracefully |
| **P5** | Auto-detect netif at nw_split init | Zero Kconfig for ownership |

P1 alone is a 3-line fix that likely resolves the immediate crash.

---

## 7. Open Questions

**Q1**: WiFi feature's `reg_rxcb(NULL)` on disconnect was intentional — it prevents
packets reaching a stale callback after disconnect. With permanent callback,
the callback handles disconnect state internally (passthrough). Is removing
the NULL safe, or should we gate it (`if !nw_split_active`)?

**Q2**: `__wrap_esp_wifi_init` reinit path calls `esp_wifi_stop()` + `deinit()`.
With the wrap guard, this returns `ESP_ERR_INVALID_STATE` when nw_split is active.
Should we instead implement suspend/resume so reinit works? Or is refusing acceptable for v1?

**Q3**: Should the WiFi feature's existing `s_pending_sta_config` caching merge with
the new wrap-layer config caching, or stay separate? They serve similar purposes.
Merging avoids double-caching; keeping separate preserves existing behavior.

<!-- %% me.br-260416nso.c %% -->
