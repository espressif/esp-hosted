# Design Overview

[Home](../../README.md) · **Design** · [Architecture](../architecture.md) · [Features](../features/README.md)

Deep-dive design docs for engineers who want the internals — protocol sequences, controller/stack layering, and tuning knobs. Start with the high-level [Architecture](../architecture.md) if you have not read it yet.

---

## Pick a topic

| Doc | What it covers |
| :--- | :--- |
| [Architecture](../architecture.md) | The core split, control vs data plane, boot sequence — the mental model. |
| [Wi-Fi Design](wifi.md) | Native vs hosted call paths; host TX/RX and co-processor execution. |
| [Bluetooth Design](bluetooth.md) | Controller, HCI transport (Hosted-HCI vs UART-HCI), NimBLE / BlueDroid hosts. |
| [SDIO Design](sdio.md) | The SDIO wire protocol: 512-byte blocks, packet-length register, stream mode, throttle. |
| [SPI Full-Duplex Design](spi-full-duplex.md) | The SPI-FD protocol: Handshake / Data-Ready GPIOs, fixed 1600-byte transactions, dummy frames. |
| [SPI Half-Duplex Design](spi-half-duplex.md) | The MCU-only SPI-HD protocol: IO modes, commands, registers, timing. |
| [UART Design](uart.md) | The UART protocol: length-prefixed framing, Wi-Fi + Bluetooth multiplexing, flow control. |
| [RPC Reference](rpc-reference.md) | Every implemented RPC request and event, with the release each was added. |
| [Performance Tuning](performance.md) | Per-chip high-performance configs and per-transport optimization. |

---

## Design principles worth internalizing

- **One bus, two planes.** Control (RPC, serialized) and data (payload, encapsulated) share the transport. Prove the transport before trusting a feature. → [Architecture](../architecture.md)
- **The host app calls as if the radio were local.** A thin shim turns a normal Wi-Fi API call into an RPC to the co-processor and back. → [Wi-Fi Design](wifi.md)
- **Only control packets are serialized.** Data packets are only encapsulated (header add/remove), avoiding endian conversion on the hot path.
- **Choose the chip for the radio you need**, then the bus for the throughput you need. → [Getting Started: MCU](../getting-started-mcu.md), [Performance Tuning](performance.md)

---

## Choosing hardware for a design

Before committing to a co-processor and bus:

- **Radio:** Classic BT needs an ESP32; Wi-Fi excludes H2/H4; OpenThread needs an 802.15.4 radio. See the co-processor support tables in [Getting Started: MCU](../getting-started-mcu.md#supported-esp-co-processors).
- **Bus:** evaluate with jumpers first, then design a PCB for SDIO or high-clock SPI. Prefer `IO_MUX` GPIOs. See the per-bus wiring in [Getting Started: MCU](../getting-started-mcu.md) / [Linux](../getting-started-linux.md), and the wire protocols in the per-bus design docs above.
- **Debugging methodology:** verify raw transport throughput before layering networking on top. See [Troubleshooting](../troubleshooting.md) and [Testing](../testing.md).
