#!/usr/bin/env python3
"""ext_coex / pta · linux_802_3 · host_py_app — eh_host ctypes demo.

Drives the CP-side external-coexistence (PTA) state via the
ext_coex feature RPC."""

import sys

import eh_host
from eh_host import ext_coex


def main() -> int:
    eh_host.init_linux_default()
    try:
        print("==> CP EXT_COEX: setting work mode to Leader Role")
        ext_coex.set_work_mode(ext_coex.LEADER_ROLE)

        gpio_pins = ext_coex.GpioSet(request=11, priority=4, grant=16, tx_line=17)
        print(
            "==> CP EXT_COEX: GPIO pins request="
            f"{gpio_pins.request}, priority={gpio_pins.priority}, "
            f"grant={gpio_pins.grant}, tx_line={gpio_pins.tx_line}"
        )
        ext_coex.set_gpio_pin(ext_coex.WIRE_3, gpio_pins)

        print("==> CP EXT_COEX: setting grant delay to 10 us")
        ext_coex.set_grant_delay(10)

        print("==> CP EXT_COEX: setting validate high to true")
        ext_coex.set_validate_high(True)

        print("Co-processor External Coexistence configuration completed.")
    except eh_host.EhHostError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    finally:
        eh_host.deinit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
