# Troubleshooting ESP-Hosted

**Table of Contents**

- [1. Raw Throughput Testing Option](#1-raw-throughput-testing-option)
- [2. Make sure Hosted code is in sync for Master and Slave](#2-make-sure-hosted-code-is-in-sync-for-master-and-slave)
- [3. Make sure GPIOs match on both the Host and Slave](#3-make-sure-gpios-match-on-both-the-host-and-slave)
- [4. ESP-Hosted Master Not Connecting to Slave](#4-esp-hosted-master-not-connecting-to-slave)
- [5. Getting `Drop Packet` Errors](#5-getting-drop-packet-errors)
- [6. References](#6-references)

## 1. Raw Throughput Testing Option

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

## 2. Make sure Hosted code is in sync for Master and Slave

The [README](../README.md) instructions will always fetch the latest
version of ESP-Hosted from the Component Registry. Generally, this
should be fine. But you can also fetch ESP-Hosted code based on a
revision to get a fixed version of the code:

For example, to fetch version 0.0.9 of ESP-Hosted Master:

```
idf.py add-dependency "espressif/esp_hosted^0.0.9"
```

To fetch version 0.0.9 of the ESP-Hosted Slave:

```
idf.py create-project-from-example "espressif/esp_hosted^0.0.9:slave"
```

This will ensure that both the Master and Slave code are fixed and in
sync for your project.

> ![NOTE]
> When you switch Hosted versions, make sure you use the same version
> of the Master and Slave code. There may be changes to the Hosted
> implementation that may make different versions of Hosted Master and
> Slave incompatible.

## 3. Make sure GPIOs match on both the Host and Slave

- check that the GPIOs you use on the Host and Slave are correct and
  are connected together as expected
- verify that the GPIO values you set in `menuconfig` match the
  hardware GPIOs you are actually using
- make sure you are not using incompatible GPIOs:
  - on the ESP32, some GPIOs are input only and cannot be used for
    output
  - on the ESP32 and ESP32-C6, the GPIOs used for SDIO are fixed and
    cannot be changed

## 4. ESP-Hosted Master Not Connecting to Slave

If you see the following error on the ESP-Hosted Master console using the SPI Interface:

```
E (10645) transport: Not able to connect with ESP-Hosted slave device
```

or this error on the ESP-Hosted Master console using the SDIO Interface:

```
E (1735) sdmmc_common: sdmmc_init_ocr: send_op_cond (1) returned 0x107
```

It means that something is wrong with the SPI or SDIO connection and
the Host cannot communicate with the slave.

- check your physical GPIO signals and verify that they are connected
- make sure you have selected the same transports for the slave and
  host (both are using the same SPI or SDIO interface)
- verify that the physical GPIO signals is the same as those assigned
  to the system using `Menuconfig` on both the Host and Slave
- if you selected SDIO as the interface and your host is a ESP32,
  there may be conflict with the GPIO used to bootstrap the ESP32 and
  used in SDIO. See "Conflicts Between Bootstrap and SDIO on DAT2" in
  [References](#6-references) for more information
- for SDIO, verify that pull-ups and other signalling requirments
  (short, shielded connections) are also met. See the [SDIO
  interface](sdio.md) page for more information on SDIO requirements

## 5. Getting `Drop Packet` Errors

For the SPI interface, if you see an error similar to this:

```
I (478522) spi: rcvd_crc[30224] != exp_crc[36043], drop pkt
```

Your SPI interface is facing signal integrity errors.

- try reducing the SPI `CLK` frequency (using `Menuconfig`). If the
  problem goes away, it indicates that there is an issue with the
  physcial SPI signals
- use an oscilloscope to check the physical signals on the SPI
  interface for noise, ringing, etc. that may affect the signals

## 6. References

- Conflicts Between Bootstrap and SDIO on DAT2:
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html#conflicts-between-bootstrap-and-sdio-on-dat2)
