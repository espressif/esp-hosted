# freertos_shim

Port-layer shim that satisfies `#include <FreeRTOS.h>` / `<queue.h>`
from vendored IDF-internal headers (`esp_private/wifi.h`,
`lwip/port/freertos/`) on non-FreeRTOS hosts.

## Layering

```
            [ examples / features / compat ]    ← eh_port_* only,
                                                   no FreeRTOS calls
                          │
                          ▼
                    [ port API ]                 ← eh_port_* primitives
                          │
                          ▼
              [ port/host/<impl> ]               ← maps eh_port_* to
                                                   pthread / FreeRTOS / CMSIS
                          │
                          ▼
        [ port/esp_idf_port/freertos_shim ]     ← THIS SLOT.  Only
                                                   reached by vendored
                                                   IDF-internal headers
                                                   that #include FreeRTOS.h
                                                   directly.
```

Upper layers should NEVER include `FreeRTOS.h` / `queue.h` / `task.h` /
`semphr.h` / `event_groups.h` directly.  Use `eh_port_*` instead.

## Per-port enablement

| Port | This slot | Source of FreeRTOS headers |
|------|-----------|---------------------------|
| `esp_idf` | OFF (early return) | IDF supplies real FreeRTOS |
| `posix` / linux_user | **ON** — typedef-only stubs | this dir |
| `freertos` (future) | OFF | integrator's toolchain (CMSIS / native FreeRTOS) |
| `cmsis_freertos` (future) | OFF | CMSIS-FreeRTOS via toolchain |

## What's in `include/`

Typedef-only stubs.  No function bodies.  If a new TU starts
referencing a missing FreeRTOS function (xQueueSend, vTaskDelay, …),
the link error is the **right** signal — the TU is wrongly compiled
on a non-FreeRTOS host.  Two correct fixes:

1. Gate the TU off non-IDF / non-FreeRTOS builds.
2. Add a port-backed wrapper here that delegates to `eh_port_*`.

Adding raw FreeRTOS function bodies here is wrong — that would
reintroduce the dependency we explicitly broke.

## Files

- `FreeRTOS.h` — `portMAX_DELAY`, `pdMS_TO_TICKS`, `pdTRUE/FALSE`.
                 Pulls `os_header.h` from the utils slot for
                 `BaseType_t` / `UBaseType_t` / `TickType_t`.
- `queue.h`    — opaque `QueueHandle_t` typedef.

Add only what's referenced by an actual vendored header.  Don't
pre-emptively stub `task.h` / `semphr.h` / `event_groups.h` until
something needs them.
