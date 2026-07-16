# GPIO Expander

[Home](../../README.md) · [Getting Started: Linux](../getting-started-linux.md) · [Getting Started: MCU](../getting-started-mcu.md) · [Troubleshooting](../troubleshooting.md)

Drive and read the co-processor's GPIOs from the host over the existing transport link (SPI, SDIO, or UART). The co-processor becomes a virtual GPIO expander — extra digital I/O without spending host pins or adding a dedicated expander chip.

---

## Host support

| Linux host | MCU host |
| :---: | :---: |
| Yes | Yes |

Typical uses: driving LEDs / status indicators, reading buttons or sensor lines, and resetting or enabling peripherals wired to the co-processor board.

---

## How it works

GPIO calls on the host are sent as RPC requests over the transport to the co-processor, which executes them against its own GPIO driver. The API mirrors the standard ESP-IDF GPIO driver and lives in `esp_hosted_cp_gpio.h`:

- `esp_hosted_cp_gpio_config()` — mode (input/output), pull-up/down, interrupt type.
- `esp_hosted_cp_gpio_set_level()` — set an output pin.
- `esp_hosted_cp_gpio_get_level()` — read an input pin.
- `esp_hosted_cp_gpio_set_direction()` — change pin direction.
- …and more.

> [!WARNING]
> **Safety guard:** the co-processor firmware rejects any host request to control a pin used by the active SPI, SDIO, or UART transport, so the host cannot accidentally (or maliciously) break its own communication link.

---

## Enable / disable

Check the option on **both** sides in `menuconfig` (host `ESP_HOSTED_HOST_FEAT_GPIO_EXP`, coprocessor `ESP_HOSTED_CP_FEAT_GPIO_EXP`); uncheck it to disable.

```text
Host:
Component config
└── ESP-Hosted
     └── [*] Enable GPIO Expander feature on host           <------- Enable this

Co-processor:
Example Configuration
└── [*] Enable GPIO Expander support                        <------- Enable this
         (host can control slave GPIOs)
```

---

## Start here

- [GPIO Expander](../../examples/gpio_expander/README.md) — configure a pin and toggle it from the host.

---

## See also

- [OTA & System](ota-and-system.md) · [Getting Started: MCU](../getting-started-mcu.md)
