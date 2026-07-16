# Feature Overview

[Home](../../README.md) · [Getting Started: Linux](../getting-started-linux.md) · [Getting Started: MCU](../getting-started-mcu.md) · [Troubleshooting](../troubleshooting.md)

Come here **after** a first example works (see [Getting Started](../getting-started-mcu.md)). For each feature this answers: does it exist, which host path fits, and the best example to start from.

---

## What runs where

| Feature | Linux | MCU | Page |
| :--- | :---: | :---: | :--- |
| Wi-Fi (STA / SoftAP / AP+STA / Scan) | Yes | Yes | [Wi-Fi](wifi.md) |
| Bluetooth / BLE | Yes | Yes | [Bluetooth](bluetooth.md) |
| OpenThread | No | Yes | [OpenThread](openthread.md) |
| Network Split | Yes | Yes | [Network Split](network-split.md) |
| Host Power Save | — | Yes | [Power Save](power-save.md) |
| GPIO Expander | Yes | Yes | [GPIO Expander](gpio-expander.md) |
| OTA & System | Yes | Yes | [OTA & System](ota-and-system.md) |

> [!NOTE]
> OpenThread is MCU-host only. Bluetooth/BLE works on both hosts — on Linux it runs through the kernel module and a normal host stack such as BlueZ (no ESP-Hosted BT example needed), see [Bluetooth](bluetooth.md).

---

## Reading order

If you are unsure where to begin:

1. Get one transport path working first — [Wi-Fi Station](../../examples/wifi/sta/README.md).
2. Pick the feature you care about from the table above.
3. Open that feature page — each links straight to its best starting example.
4. Stuck? Check the [Getting Started: MCU](../getting-started-mcu.md) or [Troubleshooting](../troubleshooting.md).

---

## Quick summaries

**Wi-Fi** — Station, SoftAP, AP+STA, and scan on both hosts. On Linux, control flows through the ESP-Hosted RPC path (not a native `wlan0`). → [Wi-Fi](wifi.md)

**Bluetooth / BLE** — On Linux, the kernel module exposes a standard HCI interface and you use a normal host stack like BlueZ. On MCU, the host runs the BT stack (NimBLE or BlueDroid) and the co-processor is the controller over Hosted-HCI or standard UART-HCI. Classic BT needs an ESP32 co-processor. → [Bluetooth](bluetooth.md)

**OpenThread** — MCU host runs the Thread stack; a separate UART-connected RCP provides the 802.15.4 radio. → [OpenThread](openthread.md)

**Network Split** — Split networking duties between host and co-processor; route selected traffic to whichever side should handle it. → [Network Split](network-split.md)

**Host Power Save** — Coordinate host sleep/wake with the co-processor (pairs with network split). → [Power Save](power-save.md)

**GPIO Expander** — Drive and read the co-processor's GPIOs from the host over RPC. → [GPIO Expander](gpio-expander.md)

**OTA & System** — Query firmware version, receive hosted events, configure transport, monitor memory, and OTA-update the co-processor. → [OTA & System](ota-and-system.md)
