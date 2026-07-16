# OTA & System

[Home](../../README.md) · [Getting Started: Linux](../getting-started-linux.md) · [Getting Started: MCU](../getting-started-mcu.md) · [Troubleshooting](../troubleshooting.md)

The system features are the housekeeping around the link: ask the co-processor what firmware it runs, receive events from it, tune the transport, watch memory, and update the co-processor over the air. All of it works from either host.

---

## Host support

| Linux host | MCU host |
| :---: | :---: |
| Yes | Yes |

---

## What's in here

**Firmware version query** — read the co-processor's firmware version from the host over RPC, so your application can verify compatibility or gate features on a minimum version.

**Hosted events** — the co-processor pushes asynchronous events to the host (connection state, ready notifications, and other status). Your app subscribes and reacts instead of polling.

**Transport configuration** — configure and initialise the bus between host and co-processor (SPI / SDIO / SPI half-duplex / UART) and its parameters. This is the setup layer every other feature rides on; see the [Getting Started: MCU](../getting-started-mcu.md) for the physical side and [Architecture](../architecture.md) for the stack.

**Memory monitor** — track host and co-processor memory / diagnostics over time, handy when validating a footprint-constrained build.

**Co-processor OTA** — update the co-processor firmware over the air, driven entirely from the host across the existing transport (no separate flashing rig). Available on both Linux and MCU hosts.

---

## Enable / disable

These are always-available system capabilities — there is **no menuconfig toggle** to enable them. Version query, hosted events, transport config, memory monitor, and co-processor OTA are all reached through the ESP-Hosted RPC control path once `esp_hosted_init()` has run.

---

## Start here

- [Hosted Events](../../examples/system/hosted_events/README.md) — subscribe to and handle co-processor events.
- [API Exerciser](../../examples/system/api_exerciser/README.md) — exercise version query, transport config, and other system RPCs.
- [Memory Monitor](../../examples/mem_monitor/README.md) — watch memory usage on both sides.
- [Coprocessor OTA](../../examples/ota/coprocessor_ota/README.md) — OTA-update the co-processor from the host.

---

## See also

- [Architecture](../architecture.md) — where the transport and RPC layers sit.
- [GPIO Expander](gpio-expander.md) · [Getting Started: MCU](../getting-started-mcu.md)
