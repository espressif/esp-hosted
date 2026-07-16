# Porting

[Home](../README.md) · [Getting Started: Linux](getting-started-linux.md) · [Getting Started: MCU](getting-started-mcu.md) · [Troubleshooting](troubleshooting.md)

ESP-Hosted is designed to run on top of a thin **port layer**. Everything above it — the
control path, Wi-Fi, Bluetooth, and the network split — is host-agnostic. Porting means
teaching that layer about *your* bus, GPIOs, and OS primitives, and then running an
**unchanged** example on top of it.

The port layer lives under `port/`:

- `port/os/{posix,idf,stm32}` — OS abstractions (tasks, queues, sync, timers, GPIO, DMA)
  behind the `eh_host_port_*` headers in `port/os/include/`.
- `port/idf_components/` — pre-ported ESP-IDF components (`esp_event`, `esp_netif`, `lwip`, …)
  so a non-IDF host does not have to reinvent the networking glue the stack expects.

One invariant runs through every path on this page:

> [!IMPORTANT]
> **Verify the transport works before you layer features on top.** On a new board most
> bring-up failures are electrical — lead length, clock skew, a wrong reset pin, or (for
> SDIO) missing pull-ups. Prove the raw link first, then the control path, then Wi-Fi.
> See [Getting Started: MCU](getting-started-mcu.md) before you debug software.

The rest of the page has two parallel sections — Linux and MCU — with the same
sub-structure, so a mental model built on one transfers to the other.

---

## Linux host porting

For any Linux SBC that is **not** the reference Raspberry Pi. The transport is a kernel
module; the goal is to make that module bind to your board's bus, then run an existing
Linux user-space example as-is.

> [!WARNING]
> Do **not** start by porting the example app. Start by porting the transport layer (the
> kernel module and its load scripts).

### What's already provided

| Piece | Location |
| :---- | :------- |
| Kernel-module transport (`esp32_spi.ko` / `esp32_sdio.ko`) | `host/linux/eh_host_linux_kmod/driver/` |
| Build / load / unload / status scripts | `host/linux/eh_host_linux_kmod/scripts/` |
| Reference board port hook | `host/linux/eh_host_linux_kmod/scripts/port_rpi.sh` |
| Device-tree assets (e.g. `spidev_disabler.dts`) | `host/linux/eh_host_linux_kmod/device_tree/` |
| Linux user-space examples | `examples/*/linux_802_3_host/` |

Board-specific logic is expected to live in a `port_<board>.sh` file that `load.sh` sources
via the `PORT` environment variable (`PORT=rpi` is the default). Every port file must define
`port_defaults`, `port_dt_overlay_spi`, `port_pinmux_bt_uart_2pin`,
`port_pinmux_bt_uart_4pin`, and `port_cpu_perf` — a no-op body is fine for functions your
board does not need.

### Hardware connections

**Power.** Use the exact adapter and cable your SBC expects. An underpowered rail makes the
peripheral bus underperform or fail intermittently. The ESP co-processor can be powered over
its microUSB/USB-C during bring-up.

**Host-side GPIOs.** Map these to your SoC's pins (in its device tree / pinctrl) and wire
them to the co-processor:

| Transport | Signals |
| :-------- | :------ |
| SDIO | `CLK`, `CMD`, `DAT0`, `DAT1`, `DAT2`, `DAT3` |
| SPI full-duplex | `CS`, `CLK`, `MISO`, `MOSI`, plus two spare GPIOs for `Handshake` and `Data Ready` |
| Reset | one spare GPIO driving the co-processor's `EN`/reset (module param `resetpin`, or a DT reset-gpio) |

**Slave-side (ESP co-processor) GPIOs.**

- **SPI**: `MISO`/`MOSI`/`CLK`/`CS` sit on the ESP `IO_MUX` and can be reassigned with a
  small GPIO-matrix routing overhead. `Handshake` and `Data Ready` may use any free GPIO.
- **SDIO**: fixed GPIOs on ESP32/ESP32-C6 — not reassignable.
- **UART**: 4-pin is recommended and fully configurable on ESP32/ESP32-C3/ESP32-S3; ESP32-C2
  and ESP32-C6 offer a 2-pin variant. You can go from 4-pin to 2-pin by disabling hardware
  flow control on **both** the co-processor and the host.

**Pull-ups.**

- SPI/UART: pull `CS` externally.
- SDIO: add external pull-ups of at least **10 kOhm** on `CMD` and `DAT0`–`DAT3` (the
  reference PCB uses **51 kOhm**). Exact values are chip-dependent — see the ESP-IDF
  [SD pull-up requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html).

**Device tree / board config.** The host device tree is *your* responsibility; ESP-Hosted
assumes it is already correct. Check your SoC datasheet for the peripheral instance and its
supported GPIOs, and cross-check min/max peripheral frequency and (for SPI) the timing
phases. For SDIO, add `broken-cd;` to the controller node so the ESP co-processor is
hot-pluggable.

> [!NOTE]
> Getting a Linux SBC and its peripheral drivers up is out of ESP-Hosted's scope — every
> platform has its own drivers and its own way of describing hardware. Most "porting bugs"
> reported to the project turn out to be host device-tree issues, not ESP-Hosted defects.

See [SPI Full-Duplex](getting-started-mcu.md#2-spi-full-duplex), [SDIO](getting-started-mcu.md#1-sdio), and
[UART](getting-started-mcu.md#4-uart) for exact wiring.

### Software porting

**Build system / toolchain.** The kernel module is tested against Linux **4.19.x** and
should build on lower versions too. Bluetooth over SPI (`HCI_SPI`) needs kernel **>= 4.7.0**.
Build against your kernel headers with `build.sh`:

```sh
cd host/linux/eh_host_linux_kmod/scripts
./build.sh --bus spi --kernel /path/to/kernel/build --arch arm64
```

Use `--bus sdio` for SDIO. For a cross build, pass `--cross-compile <prefix>` (or set the
`CROSS_COMPILE`, `KERNEL`, and `ARCH` make variables directly — `target` is `spi` or `sdio`,
defaulting to `sdio`):

```sh
make target=spi ARCH=arm64 \
  CROSS_COMPILE=/opt/toolchain/bin/aarch64-linux-gnu- \
  KERNEL=/path/to/kernel/build
```

**Transport driver (board port).** Copy `port_rpi.sh` to `port_<your_board>.sh` and adjust
the five functions for your hardware — GPIO defaults in `port_defaults`, the SPI overlay in
`port_dt_overlay_spi`, UART muxing in the two `port_pinmux_bt_uart_*pin` helpers, and the CPU
governor knob in `port_cpu_perf`. Then load through it:

```sh
PORT=<your_board> ./load.sh --bus spi \
  --reset-gpio <gpio> --clock-mhz <mhz> \
  --spi-bus <n> --spi-cs <n> --spi-mode <1|2|3> \
  --spi-handshake <gpio> --spi-dataready <gpio>
```

For SDIO, `--reset-gpio` and `--clock-mhz` are the usual knobs. `load.sh` caps the clock at
**40 MHz** for SPI and **50 MHz** for SDIO.

**OS glue.** On Linux the kernel module *is* the OS integration, so the porting work is
mostly freeing the bus and mapping GPIO numbers:

- **Free the SPI controller.** The default `spidev` driver must be disabled so `esp32_spi`
  can claim the chip select. `port_dt_overlay_spi` applies `spidev_disabler` via `dtc` +
  `dtoverlay`; confirm `/dev/spidevX.Y` for your bus/CS disappears afterward.
- **Map GPIO numbers.** Kernel-visible GPIO numbers may differ from header pin numbers.
  Raspberry Pi OS renumbered GPIOs in 2024 (gpiochip base moved to 512), so the reference
  numbers shifted:

  | Function | Old GPIO | New GPIO |
  | :------- | :------: | :------: |
  | Reset | 6 | 518 |
  | SPI Handshake | 22 | 534 |
  | SPI Data Ready | 27 | 539 |

  Discover the mapping on your board with `cat /sys/kernel/debug/gpio` and pass the resolved
  numbers to `load.sh`.

### Bring-up order

1. **Transport up.** Module builds cleanly for your kernel, loads without crashing
   (`insmod` succeeds, appears in `lsmod`), and the co-processor init/first event shows up in
   `dmesg`. Optionally run a raw-throughput test in each direction to confirm the link is
   healthy (reboot both ends when you change direction).
2. **Control path up.** Run the [Get CP FW Version](../examples/system/get_cp_fw_version/README.md)
   example. Success here proves the transport and the control-plane flow.
3. **Wi-Fi up.** Run [Wi-Fi Station](../examples/wifi/sta/README.md). The port is stable when
   this works **without** board-specific hacks inside the user-space app.

### Common pitfalls

- **Mismatched firmware.** SDIO firmware on the co-processor with an SPI kernel module on the
  host (or vice-versa) simply never talks. Verify both ends use the same bus.
- **Wrong GPIO numbers.** Especially after the RPi OS 2024 renumbering — always confirm via
  `/sys/kernel/debug/gpio`.
- **`spidev` not disabled.** The host driver cannot claim the CS line; check that
  `/dev/spidevX.Y` is gone.
- **Jumper wiring.** Keep leads short (<= 6 cm for SPI) and equal length; add extra grounds.
  Long/unequal leads cause timing failures that look like software bugs.
- **Clock / mode.** If only the first event arrives (or nothing), lower the clock (down to
  1 MHz), try a different SPI mode on **both** ends, and confirm `Handshake`/`Data Ready` are
  wired as interrupts. Persistent failures point to SPI timing tuning (logic analyzer
  recommended).
- **Missing SDIO pull-ups.** No/weak pull-ups on `CMD`/`DAT` lines is the classic SDIO
  bring-up failure.

---

## MCU host porting

For any MCU/RTOS host beyond the reference ESP-IDF path. There are two situations: a **new
ESP-IDF board or wiring** (an incremental config change) and a **non-ESP MCU host** (a real
port). The subsections below cover both.

### What's already provided

| Piece | Location |
| :---- | :------- |
| OS ports (reference implementations) | `port/os/posix`, `port/os/idf`, `port/os/stm32` |
| Port-layer headers (the surface to implement) | `port/os/include/eh_host_port*.h` |
| Host transport implementations | `host/mcu/eh_host_mcu_transport/` |
| Vendored host-support components | `port/idf_components/` (`esp_event`, `esp_netif`, `lwip`, …) |
| MCU-host examples (ESP-IDF) | `examples/*/mcu_host/` |

The `posix` and `idf` ports are working references; `stm32` is the starting point for a
bare-metal/RTOS port.

### Hardware connections

Wire the co-processor per your chosen transport, using the same host/slave GPIO and pull-up
rules as the Linux section:

- **SDIO**: `CLK`, `CMD`, `DAT0`–`DAT3`; external pull-ups (51 kOhm recommended). SDIO GPIOs
  are fixed on ESP32/ESP32-C6.
- **SPI full-duplex**: `CS`, `CLK`, `MISO`, `MOSI`, plus `Handshake` and `Data Ready`; ESP
  SPI pins are on `IO_MUX`.
- **UART**: 4-pin (recommended) or 2-pin; drop to 2-pin by disabling hardware flow control.
- **Reset**: one host GPIO to the co-processor's `EN`/reset.

Prefer ESP `IO_MUX` GPIOs on the co-processor — they have better timing and support higher
clocks. On the host MCU, verify each chosen pin can actually be used for the peripheral (some
ESP32 GPIOs are input-only and unusable for Hosted). Evaluate with jumpers first, then move
to a PCB:

| Link | Max jumper length |
| :--- | :---------------- |
| Standard / Dual SPI | 10 cm (shorter is better) |
| SDIO 1-bit | 5 cm |
| Quad SPI, SDIO 4-bit | PCB only — jumpers not supported |

Keep all leads equal length, connect as many grounds as possible, and isolate `CLK`. Add
tapping points/headers on the interface signals (and a couple of spare GPIOs) to your
prototype for logic-analyzer debugging and future power-control GPIOs. See
[SPI Full-Duplex](getting-started-mcu.md#2-spi-full-duplex), [SDIO](getting-started-mcu.md#1-sdio), and
[UART](getting-started-mcu.md#4-uart), and the general guidance in
[Getting Started: MCU](getting-started-mcu.md).

### Software porting

**New ESP-IDF board or wiring (incremental).** This is a configuration change, not a real
port. Pick a baseline example, set the transport choice, GPIOs (including reset), and any
clock/mode settings in the transport Kconfig under `host/mcu/eh_host_mcu_transport/`, and
select the **exact same transport and bus settings** on the co-processor.

> [!WARNING]
> If host and co-processor transport configs are not aligned, feature debugging is premature.

**Non-ESP MCU host (real port).** Provide enough host-side compatibility for the stack to
run on your MCU/RTOS:

1. **Implement the OS port.** Create `port/os/<rtos>` following `posix`/`idf` (for example an
   STM32/Zephyr-style port), implementing the `eh_host_port_*` primitives declared in
   `port/os/include/` — tasks, queues, sync, timers, GPIO, and DMA.
2. **Lean on the vendored components.** The pieces under `port/idf_components/` (`esp_event`,
   `esp_netif`, `lwip`, …) exist to reduce how much host-side glue you must invent.
3. **Implement ONE transport driver.** Bring up **SPI full-duplex** first — it is the
   simplest and most widely available bus. SDIO is a good second step if your MCU has a
   suitable controller.

> [!WARNING]
> Do not try to port all buses at once. Prove one transport end-to-end first.

### Bring-up order

1. **Transport up.** One transport comes up reliably and the host receives the co-processor
   init event. Use the built-in raw-throughput option to confirm the link and get a rough
   throughput number before any networking is involved.
2. **Control path up.** Run [Get CP FW Version](../examples/system/get_cp_fw_version/README.md)
   — this proves the host transport, host-stack integration, and the control plane.
3. **Wi-Fi up.** Run [Wi-Fi Station](../examples/wifi/sta/README.md). Only after that attempt
   Network Split, power save, or other feature-heavy flows. The port is stable when Wi-Fi
   Station works without per-run hacks.

### Common pitfalls

- **Config mismatch.** Host and co-processor must select the same transport and bus settings;
  this is the single most common cause of a dead link.
- **Porting too much at once.** Get one transport working before adding a second.
- **Networking before raw throughput.** If raw throughput is poor or unstable, the transport
  is not correctly ported — fix that before blaming lwip/networking.
- **Unusable GPIOs.** Confirm each host and co-processor GPIO can serve the peripheral role
  (input-only ESP32 pins, fixed SDIO pins, etc.).
- **Signal integrity.** Long/unequal jumpers and missing SDIO pull-ups produce
  software-looking failures; use a PCB and proper pull-ups for SDIO.

---

**Next:** wire your bus in the [Getting Started: MCU](getting-started-mcu.md) guide, follow
[Getting Started: Linux](getting-started-linux.md) or
[Getting Started: MCU](getting-started-mcu.md), and if something misbehaves see
[Troubleshooting](troubleshooting.md).
