#!/usr/bin/env python3
"""gpio_expander · linux_802_3 · host_py_app — eh_host ctypes demo.

Drives a CP-side GPIO via the gpio_exp RPC: configure output,
toggle, reconfigure input, read."""

import sys
import time

import eh_host
from eh_host import gpio_exp


SLAVE_GPIO_PIN = 2


def _toggle_output(label: str, mode: int, pull_up: int = 0) -> None:
    cfg = gpio_exp.GpioConfig(
        pin_bit_mask=(1 << SLAVE_GPIO_PIN),
        mode=mode,
        pull_up_en=pull_up,
        pull_down_en=0,
        intr_type=0,
    )
    print(f"---- {label} ----")
    gpio_exp.config(cfg)
    print(f"Setting GPIO {SLAVE_GPIO_PIN} LOW")
    gpio_exp.set_level(SLAVE_GPIO_PIN, 0)
    time.sleep(0.5)
    print(f"Setting GPIO {SLAVE_GPIO_PIN} HIGH")
    gpio_exp.set_level(SLAVE_GPIO_PIN, 1)
    time.sleep(0.5)


def _read_input(label: str, pull_up: int) -> None:
    cfg = gpio_exp.GpioConfig(
        pin_bit_mask=(1 << SLAVE_GPIO_PIN),
        mode=gpio_exp.GPIO_MODE_INPUT,
        pull_up_en=pull_up,
        pull_down_en=0,
        intr_type=0,
    )
    print(f"---- {label} ----")
    gpio_exp.config(cfg)
    level = gpio_exp.get_level(SLAVE_GPIO_PIN)
    print(f"GPIO {SLAVE_GPIO_PIN} level: {level}")


def main() -> int:
    eh_host.init_linux_default()
    try:
        _toggle_output("Demo 1: Standard Output", gpio_exp.GPIO_MODE_OUTPUT)
        _toggle_output(
            "Demo 2: Open-Drain Output", gpio_exp.GPIO_MODE_OUTPUT_OD, pull_up=1
        )
        _read_input("Demo 3: Input with Pull-Up", pull_up=1)
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
