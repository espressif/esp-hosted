# ESP-Hosted SDIO Operation

**Table of Contents**

- [1. Introduction](#1-introduction)
- [2. SDIO Configuration](#2-sdio-configuration)
  - [2.1. Extra GPIO Signals Required](#21-extra-gpio-signals-required)
- [3. Hardware Considerations](#3-hardware-considerations)
  - [3.1. Pull-up Resistors](#31-pull-up-resistors)
  - [3.2 PBC Traces](#32-pbc-traces)
  - [3.3. Conflicts between SDIO and ESP hardware.](#33-conflicts-between-sdio-and-esp-hardware)
  - [3.4. Testing the SDIO Connection](#34-testing-the-sdio-connection)
  - [3.5. Summary of SDIO Hardware Requirements](#35-summary-of-sdio-hardware-requirements)
  - [4. References](#4-references)

## 1. Introduction

SDIO is a high-speed bus. It uses the same SDMMC hardware protocol
used for SD Cards, but with its own set of commands for communicating
with SDIO aware peripherals.

> [!NOTE]
> Only some ESP32 chips support the SDIO Protocol:
>
> - SDIO as Slave: ESP32-C6
> - SDIO as Master: ESP32, ESP32-S3, ESP32-P4

## 2. SDIO Configuration

For the ESP32 and ESP32-C6, the pin assignments for SDIO are fixed:

| Signal | ESP32 GPIO | ESP32-C6 GPIO |
| :----  | :---:      | :---:         |
| CLK    |    14      |       19      |
| CMD    |    15      |       18      |
| DAT0   |     2      |       20      |
| DAT1   |     4      |       21      |
| DAT2   |    12      |       22      |
| DAT3   |    13      |       23      |

For the ESP32-S3 and ESP32-P4, any GPIO pin can be configured for use
in SDIO.

> [!NOTE]
> Check the ESP chip documentation for GPIO limitations that may
> prevent some GPIOs from being used for SDIO.

To enable SDIO on the Host and Slave using `Menuconfig`:

1. On Host: **Component config** ---> **ESP-Hosted config** --->
   **Transport layer** and choose **SDIO**.
2. On Slave: **Example configuration** ---> **Transport layer** and
   choose **SDIO**.

### 2.1. Extra GPIO Signals Required

Extra GPIO signal are required for SDIO on Hosted. These can be
assigned to any free GPIO pins:

- `Reset` signal: an output signal from the host to the slave. When
  asserted, the host resets the slave. This is done when ESP-Hosted is
  started on the host, to synchronise the state of the host and slave.

> [!NOTE]
> The `Reset` signal can be configured to connect to the `EN` or `RST`
> pin on the slave, or assigned to a GPIO pin on the slave.
>
> To configure this, use `Menuconfig` on the Slave: **Example
> configuration** ---> **SDIO Configuration** and set **Slave GPIO pin
> to reset itself**.

## 3. Hardware Considerations

SDIO has several hardware requirements that must be met for proper
operation.

### 3.1. Pull-up Resistors

SDIO requires external pull-up resistor (51 kOhm recommended) and
clean signals for proper operation. For this reason, it is not
recommended to use jumper cables. Use PCB traces to connect between a
Hosted Master and Slave.

### 3.2 PBC Traces

The PCB traces for SDIO should be equal length and kept as short as
possible. The SDIO signals, especially the `CLK` signal, should be
isolated from other signals using a ground plane to minimise
crosstalk.

If you must test SDIO using jumper cables, and you provide pull-up
resistors to the SDIO lines, you may be able to get SDIO working by
using a lower frequency and operating in 1-Bit SDIO mode. See
[3.4. Testing the SDIO Connection](#34-testing-the-sdio-connection)
for more information.

### 3.3. Conflicts between SDIO and ESP hardware.

SDIO requires pull-ups on signal lines which may conflict with the
pull state of hardware pins required for bootstrapping. For example
there is a conflict between the SDIO `DAT2` line and bootstrap
requirement for the EPS32 with the 3.3 V flash chip. This can be fixed
by burning the flash voltage selection eFuses.

> [!WARNING]
> eFuse burning is irreversible and may cause your hardware to stop
> functioning. Check the documentation in
> [4. References](#4-references) carefully to make sure that burning
> the eFuses is the correct option.

### 3.4. Testing the SDIO Connection

**Using a Lower Clock Speed**

You can use a lower clock speed to verify the connections. Start with
a clock speed between 400 kHz to 20 MHz.

To configure this, use `Menuconfig` on the Host: **Component
config** ---> **ESP-Hosted config** ---> **Hosted SDIO Configuration**
and set **SDIO Clock Freq (in kHz)**.

> [!NOTE]
> The actual clock frequency used is determined by the hardware. Use
> an oscilloscope or logic analyser to check the clock frequency.

**3.4.2. Using 1-bit SDIO Mode**

You can set the SDIO Bus Width to 1-Bit. In 1-Bit mode, only `DAT0`
and `DAT1` signals are used for data and are less affected by noise on
the signal lines. This can help you verify that the SDIO protocol is
working at the logical level, if you have issues getting 4-Bit SDIO to
work on your prototype board.

To configure this, use `Menuconfig` on the Host: **Component config**
---> **ESP-Hosted config** ---> **Hosted SDIO Configuration** --->
**SDIO Bus Width** to **1 Bit**.

> [!NOTE]
> Pull-ups are still required on `DAT2` and `DAT3` lines to prevent
> the SDIO slave from going into SPI mode upon startup.

### 3.5. Summary of SDIO Hardware Requirements

> [!IMPORTANT]
> - using jumper cables is not recommended
> - external pull-up are required. 10 kOhm resistors are recommended
> - check for conflicts between SDIO pull-up and GPIO requirements
> - for ESP32, check if eFuse burning is required

### 4. References

- ESP-IDF SD Pull-up Requirements:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html
