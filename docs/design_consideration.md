# Design Considerations and Debugging ESP-Hosted

**Table of Contents**

- [1. Choosing the Correct ESP chip as Slave](#1-choosing-the-correct-esp-chip-as-slave)
- [1.1. Using ESP chip as Hosted Master](#11-using-esp-chip-as-hosted-master)
- [2. General Hardware Considerations](#2-general-hardware-considerations)
  - [2.1. GPIOs used for interface](#21-gpios-used-for-interface)
  - [2.2. Using SPI insted of SDIO](#22-using-spi-insted-of-sdio)
  - [2.3. Whenever possible, Use `IO_MUX` GPIOs.](#23-whenever-possible-use-io_mux-gpios)
  - [2.4. Signal Length and Noise Reduction](#24-signal-length-and-noise-reduction)
- [3. General Debugging Guidelines](#3-general-debugging-guidelines)
  - [3.1. Add tapping points to your PCB prototype](#31-add-tapping-points-to-your-pcb-prototype)
  - [3.2. Tap out additional GPIO signals as testing points and future expansion](#32-tap-out-additional-gpio-signals-as-testing-points-and-future-expansion)
  - [3.3. Verifying Hosted Interface with Raw Throughput](#33-verifying-hosted-interface-with-raw-throughput)
- [4. Others](#4-others)
- [5. References](#5-references)

There are several considerations that need to be taken into account
when implementing ESP-Hosted for your system.

## 1 Choosing the Correct ESP chip as Slave

For prototyping, any ESP32 chip can be used as the slave, provided it
has the required interface (SPI, SDIO). But when creating an actual
product, it is important to select the proper ESP32 chip as a Hosted
Slave.

There are many ESP32 chips, each with difference features and
performance capabilities. Based on your product requirements
(interface to use, CPU, memory, power requirements, etc.), choose the
ESP32 chip(s) that can meet your requirements.

Use the ESP Product Selector guide to help you decide which ESP32
chips and/or modules are suitable for your product.

> [!NOTE]
> See [References](#5-references) for links to the Selector Guide
> and other links.

## 1.1 Using ESP chip as Hosted Master

The project defaults to using an ESP chip as the Hosted Master. This
is to act as a reference platform and make it easier to evaluate and
test Hosted before porting it to your MCU of choice.

## 2 General Hardware Considerations

### 2.1 GPIOs used for interface

Make sure the correct GPIOs pins on the Hosted Slave and Master are
connected together. Verify that the correct GPIOs are set-up in
`Menuconfig` for both the Slave and Master.

> [!NOTE]
> In general most ESP GPIOs can be used for input and output. But on
> the ESP32, some GPIOs can only be used for input and are not usable
> under Hosted. Check the ESP datasheet to verify the GPIOs you select
> can be used as a Hosted interface.

### 2.2 Evaluate with jumpers first

It is flexible to evaluate with jumper cables or bread board than full-fledged PCB.
In general, SPI (Standard & Dual SPI) imposes fewer hardware requirements compared to
SDIO. SPI is easier to prototype, and available on more ESP chips and
MCUs compared to SDIO.
Before going to SDIO 4 bit mode PCB, it's better to evaluate SDIO 1-Bit mode.

Once you evaluate the solution on jumper cables, you can move to PCB solutions with same or high performance transport.

###### Jumper cable considerations
- Use high quality jumper cables
- Use jumper cables as small as possible. you can cut and solder the joints if need be.
- Use equal length jumper cables for all the connections
- Grounds: Connect as many grounds as possible, this lowers the interference.
- Jumper cable lengths
  - Standard SPI: At max 10cm, lower the better
  - Dual SPI: At max 10cm, lower the better
  - SDIO 1 Bit: At max 5cm, lower the better
  - Quad SPI : jumpers not supported, only PCB
  - SDIO 4 Bit: Jumpers not supported, only PCB

### 2.3 Whenever possible, Use `IO_MUX` GPIOs.

In general, ESP peripheral interfaces can be assigned to any available
GPIO through a multiplexer. But some ESPs have dedicated GPIOs for
peripherals (`IO_MUX`). These `IO_MUX` GPIOs have better timing
characteristics and support higher frequencies. They should be use
when possible to minimise timing and skew issues when using the
interface for Hosted.

> [!NOTE]
> The SDIO interface on the ESP32 and ESP32-C6 have fixed GPIO
> assignments and cannot be changed.

### 2.4 Signal Length and Noise Reduction

For best performance, a PCB with traces should be used to connect the
Hosted Slave and Master. For prototyping, jumper cables can be used,
but may only work at a lower `CLK` frequency.

In general, keep the cable and PCB traces short and of the same
length, to minimise propogation delay and clock skew:

- for SPI, keep them to 10 cm or less
- for SDIO, keep them to 5 cm or less

Isolate the interface signals, expecially the `CLK` signal, from other
signals. For PCBs, surround the signal swith a ground plane, and keep
the `CLK` signal clean by not routing it close to other high frequency
signals.

For jumper cables, you can try surrounding the signals, especially the
`CLK` signal, with grounded wires to shield them from interference.

> [!NOTE]
> For SDIO, external pull-up resistors (recommended value: 51 kOhms)
> are required. Using jumper cable are **not** recommended for SDIO. You
> may be able to get SDIO working with jumper cables by using a lower
> `CLK` frequency and using 1-bit SDIO mode.

> [!NOTE]
> Also check the Hosted documentation for SPI and SDIO for more
> information and guidelines on the interfaces.

## 3 General Debugging Guidelines

### 3.1 Add tapping points to your prototype

Adding tapping points or headers to the Hosted interface signals on
your prototype will make it easier to check whether the Hosted
interface is working as expected.

### 3.2 Tap out additional GPIO signals as testing points and future expansion

Add tapping points to some unused GPIOs on both the Hosted Slave and
Host on your prototype PCB. This can later be use for debugging or
enhancing your own Hosted code.

For example, add your own debugging code to the Hosted Slave and
Master code to set a GPIO value when a condition is met. This GPIO can
be used to light a LED or trigger a capture on an oscilloscope or
logic analyzer, for example. This is useful for capturing rare or
intermittent conditions while testing Hosted.

In the future, Hosted may also offer newer transport options or more features, like controlling
power modes on the Host and Slave. These may require additional GPIOs
for control, so it would be good to keep some additional GPIOs
available and accesable for future use.

### 3.3 Verifying Hosted Interface with Raw Throughput

ESP-Hosted has a Raw Throughput Option to test sending of data between
the Host and Slave. This can be used to verify the hardware for signal
errors and to check the achievable throughput of Hosted.

> [!IMPORTANT]
> Use this option to verify that Hosted hardware and software are
> working as expected before involving other software layers like
> networking.

To enable the Raw Throughput Option on Slave, enter `Menuconfig` and
enable **Example Configuration** ---> **Hosted Debugging** --->
**RawTP**.

To enable the Raw Throughput Option and set Raw Throughput direction
on Host, enter `Menuconfig` and enable **Component config** --->
**ESP-Hosted config** ---> **Debug Settings** ---> **RawTP**. Set
the data transfer direction: **Host to Slave**, **Slave to Host** or
**Bidirectional**.

## 4 Others

Check the References below for links to the Product Selector, and more
detailed information on the interfaces used in Hosted. If you have
other issues with Hosted, you can check the Troubleshooting Guide.

You can also raise an Issue on the ESP-Hosted Github repository. Check
that the issue has not already been raised before submitting. The
solution to your problem may have already been provided.

## 5 References

**External Links**

- ESP Product Selector: https://products.espressif.com/
- ESP-Hosted Github Issues: https://github.com/espressif/esp-hosted/issues

**ESP-Hosted Documentation Links**

- SPI Full Duplex interface documentation: [spi_full_duplex.md](spi_full_duplex.md)
- SDIO interface documentation: [sdio.md](sdio.md)
- SPI Half Duplex interface documentation: [spi_half_duplex.md](spi_half_duplex.md)
- UART documentation: [uart.md](uart.md)
- Troubleshooting Guide: [troubleshooting.md](troubleshooting.md)
