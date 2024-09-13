# Troubleshooting ESP-Hosted

**Table of Contents**

- [1. ESP host to evaluate already has Native Wi-Fi](#1-esp-host-to-evaluate-already-has-native-wi-fi)
- [2. Raw Throughput Testing](#2-raw-throughput-testing)
- [3. Make sure Hosted code is in sync for Master and Slave](#3-make-sure-hosted-code-is-in-sync-for-master-and-slave)
- [4. Make sure GPIOs match on both the Host and Slave](#4-make-sure-gpios-match-on-both-the-host-and-slave)
- [5. ESP-Hosted Master Not Connecting to Slave](#5-esp-hosted-master-not-connecting-to-slave)
- [6. Getting `Drop Packet` Errors](#6-getting-drop-packet-errors)
- [7. References](#7-references)

## 1 ESP host to evaluate already has Native Wi-Fi

Sometimes users have two ESPs, but both having Wi-Fi native capability.
This section explains how to run ESP-Hosted-MCU on ESP host chipsets that already have native Wi-Fi support. To run ESP-Hosted-MCU on such hosts, native Wi-Fi support needs to be disabled from base ESP-IDF in use. There are alternatives to do this:

##### 1.1 Different ESP chipset types for host and slave
If host and slave not the same ESP chipset types, Wi-Fi capability can be disabled for host ESP chipset alone. Edit the ESP-IDF file
`components/soc/<soc>/include/soc/Kconfig.soc_caps.in` and change
all `WIFI` related configs to `n`. For example:

```
config SOC_WIFI_SUPPORTED
    bool
    # default y # original configuration
    default n
```

This should be done for all `SOC_WIFI_xxx` configs found in the file.

For ESP Chipsets without native Wi-FI, `SOC_WIFI_xxx` configs will be
`n` by default.


##### 1.2 Same ESP chipset types for host and slave
There is possibility that you have two chipsets to evaluate, but both are exactly same chipset type. For example, two ESP32-C3. In this case, it is a two step build, first for host and second for slave.
While building for host ESP chipset, follow above (1) and flash, monitor. Once host is flashed fine, revert all the changes and flash the slave ESP chipset.

## 2 Raw Throughput Testing

While aiming the high performance and even while assessing the solution correctness, It is crucial to understand the bottlenecks in the system.
'Raw throughput testing' is simple transport level testing, which would showcase the maximum throughput that the transport is able to achieve, right now in current set-up.
In this test, dummy data is sent from one transport end to other continously, without involving Wi-Fi, Bluetooth or any other code legs. This test can be performed in following ways:
- Host to slave (Half duplex) : dummy data to be sent from host to slave continously
- Slave to Host (Half duplex) : dummy data to be sent from slave to host continously
- Full duplex bi-directional : dummy data to be sent from both the directions simulataneously

This can verify hardware signal integrity and address porting issues. It also helps to assess the achievable throughput of the Hosted solution. It can be further optionally used for transport throughput fine-tuning.

> [!IMPORTANT]
> Use Raw throughput test to verify that Hosted hardware and software are
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

## 3 Make sure Hosted code is in sync for Master and Slave

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
sync for your project. Please ensure you use latest versions for bug-fixes

> [!NOTE]
> When you switch Hosted versions, make sure you use the same version
> of the Master and Slave code. There may be changes to the Hosted
> implementation that may make different versions of Hosted Master and
> Slave incompatible.

## 4 Make sure GPIOs match on both the Host and Slave

- Check that the GPIOs you use on the Host and Slave are correct and are connected together as expected
- Verify that the GPIO values you set in `menuconfig` match the hardware GPIOs you are actually using
- Ensure that you are not using incompatible GPIOs:
  - on the ESP32, some GPIOs are input only and cannot be used for output
  - on the ESP32 and ESP32-C6, the GPIOs used for SDIO are fixed and cannot be changed

## 5 ESP-Hosted Master Not Connecting to Slave

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

- Check your physical GPIO signals and verify that they are connected
- Ensure that you have selected the same transports for the slave and
  host (both are using the same SPI or SDIO interface).
  - It is expected that slave and host uses exact same codebase (git commit)
  - Transport configured at slave matches to that of host
  - Firmware configured with incompatible configurations also would result in issues.
- Verify that the physical GPIO signals is the same as those assigned to the system using `Menuconfig` on both the Host and Slave
- If you selected SDIO as the interface and your host is a classic ESP32, there may be conflict with the GPIO used to bootstrap the ESP32 and used in SDIO. See "Conflicts Between Bootstrap and SDIO on DAT2" in
  [References](#7-references) for more information
- for SDIO, verify that pull-ups and other signalling requirments (short, shielded connections) are also met. See the [SDIO interface](sdio.md) page for more information on SDIO requirements
- If your transport allows on jumper cables, cross-check max length of jumper cables allowed

## 6 Getting `Drop Packet` Errors

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

## 7 References

- [Conflicts Between Bootstrap and SDIO on DAT2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html#conflicts-between-bootstrap-and-sdio-on-dat2)
