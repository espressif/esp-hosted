# ESP-Hosted

ESP-Hosted is an open source solution that provides a way to use
Espressif SoCs and modules as a communication co-processor. This
solution provides wireless connectivity (Wi-Fi and BT/BLE) to the host
microprocessor or microcontroller, allowing it to communicate with
other devices.

## ESP as MCU Host branch

This branch provides ESP-Hosted functionality to any generic MCU
host. The purpose is to allow the [ESP Wi-Fi api
calls](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
to be used on hosts that don't have native Wi-Fi.

This branch showcases ESP chipsets to use as MCU hosts. For other
MCUs, a port layer is required to adapt Hosted to your MCU and OS.

## Getting started

Two extra components are required to intergrate ESP-Hosted into the
host code. Both are available in the [ESP
Registry](https://components.espressif.com/):
- WiFi Remote ([https://components.espressif.com/components/espressif/esp_wifi_remote](https://components.espressif.com/components/espressif/esp_wifi_remote))
- ESP-Hosted ([https://components.espressif.com/components/espressif/esp_hosted](https://components.espressif.com/components/espressif/esp_hosted))

WiFi Remote is an API layer that provides the standard ESP-IDF Wi-Fi
calls to the application (`esp_wifi_init()`, etc.). WiFi Remote
forwards the Wi-Fi calls to ESP-Hosted, which transports the incoming
Wi-Fi calls as remote requests to the slave.

Responses are received by ESP-Hosted and returned to WiFi Remote,
which returns the reponses to the calling app. To the calling app, it
is as if it made a standard ESP-IDF Wi-Fi api call.

Wi-Fi events are received by Hosted and send to the standard ESP-IDF
event loop on the host.

## Selecting hardware interface

Hosted can currently use the SPI or SDIO (on selected ESP chips)
interfaces. For testing Hosted, it is recommended to use the SPI
interface as it is easier to evaluate using jumper cables. SDIO and
UART are also supported. Among the interfaces, SDIO provides the
highest throughput.

### Using the SPI interface

The SPI interface can use almost any GPIO pins. But for maximum speed
and minimal delays, it is recommended to select the default SPI pins
configuration that use the dedicated `IO_MUX` pins. See these documents
for [SPI
host](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#gpio-matrix-and-io-mux)
and [SPI
slave](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#gpio-matrix-and-io-mux)
for more information.

This table summarises the recommended SPI GPIO pins for various ESP SoCs:

| GPIO | ESP-32 | ESP32-C2/C3/C6 | ESP32-S2/S3 |
| :--: |    --: |            --: |         --: |
|------|--------|----------------|-------------|
| MOSI |     13 |              7 |          11 |
| MISO |     12 |              2 |          13 |
| CLK  |     14 |              6 |          12 |
| CS   |     15 |             10 |          10 |

Besides the standard SPI `CS`, `CLK`, `MOSI` and `MISO`
signals, additional GPIOs are required. These are for `Handshake` and
`Data Ready` and can be assigned to any GPIOs.

For prototyping, it is recommended to use short wires (5 to 10 cm in
length, shorter is better) to minimise propagation delay and noise.

### Using the SDIO interface

The SDIO interface has the same pull-up requirements as the SD
interface (10 kOhm resistors are recommended). See the [SD Pull-up
Requirements](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html)
documentation for more details. Because of the pull-up requirements,
using jumper cables for SDIO is not recommended.

If you use the ESP32 as the SDIO slave, fixing the SDIO pin voltage may
be required by buring the eFuses, depending on the ESP32 module used. See
the ["Overview of
Compatability"](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sd_pullup_requirements.html#compatibility-overview-espressif-hw-sdio)
documentation.

Only the ESP32-C6 and ESP32 can be used as the SDIO slave, while the
ESP32 and ESP32-S3 can be used as the SDIO host.

On the ESP32 / ESP32-C6, the SDIO pins are fixed:

|        | ESP32 | ESP32-C6 |
| :--    |   --: |      --: |
| Signal |  GPIO |     GPIO |
|--------|-------|----------|
| CLK    |    14 |       19 |
| CMD    |    15 |       18 |
| DAT0   |     2 |       20 |
| DAT1   |     4 |       21 |
| DAT2   |    12 |       22 |
| DAT3   |    13 |       23 |

On the ESP32-S3 the GPIO pin assignments are flexible.

To summarize, **for SDIO**:

---

> - using jumper cables is not recommended.
> - external pull-up are required. 10 kOhm resistors are recommended.
> - for ESP32, check if eFuse burning is required

---

### Reset signal for the slave

The host also needs a signal to reset the slave during
initialization. This can be any GPIO on the host. This signal can be
connected to the `RST` / `EN` pin or to a GPIO pin on the slave.

This is required for any hardware interface used.

## Preparing Host to use ESP-Hosted

### Disabling native Wi-Fi support

Hosts with native Wi-Fi support (ESP32 series, for example) need to
disable it before ESP Hosted can be enabled and used on the system.

To do this, edit the ESP-IDF
`components/soc/<host soc>/include/soc/Kconfig.soc_caps.in` file and change
all `WIFI` related configs to `n`. For example:

```
config SOC_WIFI_SUPPORTED
    bool
    # default y # original configuration
    default n
```

This should be done for all `config SOC_WIFI_xxx` found in the file.

This is not required for ESP32-P4 as it has no native Wi-Fi and its
`Kconfig.soc_caps.in` reflects this configuration.

### Adding required components

To integrate the required components into the host code, add the
following dependencies to your host application `idf_component.yml`
file:

```
  espressif/esp_hosted:
    version: "*"
    pre_release: true
  espressif/esp_wifi_remote:
    version: "*"
```

ESP-Hosted is currently in pre-release, so the `pre_release: true`
line is required. Once ESP-Hosted is out of pre-release, it will not
be needed.

## Preparing Slave for ESP-Hosted

To build ESP-Hosted on the Slave, check out the slave as example code
from the [ESP-Hosted ESP
Registry](https://components.espressif.com/components/espressif/esp_hosted)
into an empty directory:

`idf.py create-project-from-example "espressif/esp_hosted:slave"`

## Configuring the Host and Slave

To configure the host and slave, run `idf.py set-target <soc>` and
`idf.py menuconfig`.

The configuration for the Host can be found at `Component config
---> ESP-Hosted config`.

The configuration for the Slave can be found at `Example
Configuration`.

Ensure that both are configured to use the same interface (SPI, SDIO)
and GPIOs have been selected to match the hardware connections. The
clock speed can be set to a lower speed (10 MHz for SPI, 20 MHz for
SDIO) first to verify the connections.

## Building and running

You can now run `idf.py build` to build both the host and slave,
`idf.py -p <uart port> flash monitor` to flash the firmware and run
the terminal monitor.
