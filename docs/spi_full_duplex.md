# ESP-Hosted SPI HD (Full Duplex) Operation

**Table of Contents**

- [1. Introduction](#1-introduction)
- [2. SPI FD Configuration](#2-spi-fd-configuration)
  - [2.1. Clock and Phase](#21-clock-and-phase)
  - [2.2. GPIO configuration for SPI FD](#22-gpio-configuration-for-spi-fd)
  - [2.3. Extra GPIO Signals Required](#23-extra-gpio-signals-required)
- [3. Hardware Considerations](#3-hardware-considerations)
  - [3.1. Jumper Wires](#31-jumper-wires)
  - [3.2 PCB Design](#32-pcb-design)
  - [3.3 Testing the SPI Connection](#33-testing-the-spi-connection)
  - [4. References](#4-references)

## 1. Introduction

The ESP32 family of chips support the standard SPI FD (Full Duplex)
Mode Protocol.

In this mode of operation, SPI uses 2 data lines to transfer data to
the slave and from the slave at the same time (full duplex) during an
SPI transaction.

## 2. SPI FD Configuration

To enable SPI FD on the Host and Slave using `Menuconfig`:

1. On Host: **Component config** ---> **ESP-Hosted config** --->
   **Transport layer** and choose **SPI Full-duplex**.
2. On Slave: **Example configuration** ---> **Transport layer** and
   choose **SPI Full-duplex**.

### 2.1. Clock and Phase

> [!NOTE]
> The standard SPI CPOL clock and CPHA phase must be the same on both
> the host and slave for the protocol to work.

### 2.2. GPIO configuration for SPI FD

The SPI interface can use almost any GPIO pins. For maximum speed and
minimal delays, it is recommended to select the SPI pin configuration
that uses the dedicated `IO_MUX` pins. See
[4. References](#4-references) for more information.

This table summarises the recommended SPI GPIO pins for various ESP SoCs:

| GPIO | ESP32 | ESP32-C2/C3/C6 | ESP32-S2/S3 |
| :--- |  :--: |           :--: |        :--: |
| MOSI |    13 |              7 |          11 |
| MISO |    12 |              2 |          13 |
| CLK  |    14 |              6 |          12 |
| CS   |    15 |             10 |          10 |

> [!NOTE]
> Check the ESP chip documentation for GPIO limitations that may
> prevent some GPIOs from being used for SPI.

### 2.3. Extra GPIO Signals Required

Extra GPIO signal are required for SPI FD on Hosted and can be
assigned to any free GPIO pins:

- `Handshake` signal: an output signal from the slave to the
  host. When asserted, it acts like the UART CTS (Clear to Send),
  telling the host that the slave is ready for a SPI transaction. The
  host should not perform a SPI transaction if the `Handshake` signal
  is deasserted.
- `Data Ready` signal: an output signal from the slave to the
  host. When asserted, the slave is telling the host that it has data
  to send. The host should perform a SPI transaction to fetch the data
  from the slave.
- `Reset` signal: an output signal from the host to the slave. When
  asserted, the host resets the slave. This is done when ESP-Hosted is
  started on the host, to synchronise the state of the host and slave.

> [!NOTE]
> The `Reset` signal can be configured to connect to the `EN` or `RST`
> pin on the slave, or assigned to a GPIO pin on the slave.
>
> To configure this, use `Menuconfig` on the Slave: **Example
> configuration** ---> **SPI FUll-duplex Configuration** ---> **Host
> SPI GPIOs** and set **Slave GPIO pin to reset itself**.


## 3. Hardware Considerations

### 3.1. Jumper Wires

While jumper wires can be used to test SPI, it is recommended to use
short wires (5 to 10 cm in length, shorter is better) to
minimise propagation delay and noise on the signals.

### 3.2 PCB Design

The PCB traces for SPI should be equal length and kept as short as
possible. The signals, especially the `CLK` signal, should be isolated
from other signals using a ground plane to minimise crosstalk.

### 3.3 Testing the SPI Connection

**Using a Lower Clock Speed**

You can use a lower clock speed to verify the connections. For SPI,
you can start with 10 MHz or lower.

To configure this, use `Menuconfig` on the Host: **Component
config** ---> **ESP-Hosted config** ---> **SPI Configuration**
and set **SPI Clock Freq (MHz)**.

> [!NOTE]
> The actual clock frequency used is determined by the hardware. Use
> an oscilloscope to check the clock frequency.

### 4. References

- GPIO Matrix and IO_MUX considerations for SPI Master: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#gpio-matrix-and-io-mux
- GPIO Matrix and IO_MUX considerations for SPI Slave: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#gpio-matrix-and-io-mux
