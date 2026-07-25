# SPDX-License-Identifier: Apache-2.0
"""Hosted Bluetooth (NimBLE over ESP-Hosted) on the emulator.

The host (P4) runs the NimBLE stack; the CP (C6) is a controller-only
`hosted_hci` bridge that relays HCI over the transport. Success = the host
reaches the advertising state, which only happens after the full controller
init round-trip completes — so it is the "controller works" signal.

Advertising is reachable with esp-emu's built-in virtual BLE controller (no
external peer / --ble-hci). Connect + GATT would additionally need Bumble.

Coverage: bleprph_minimal exercises every transport (SDIO carries the H4 type
byte in the frame header; UART/SPI-FD/SPI-HD re-frame it via
eh_host_mcu_hci_take_type). bleprph_gatt confirms a second NimBLE scenario over
the (already transport-proven) SDIO path.
"""

import pytest

from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

ADVERTISING = r'advertising as '

# (example, transport): minimal proves all wires; gatt proves a 2nd scenario.
BT_CELLS = [
    ("bleprph_minimal", "sdio"),
    ("bleprph_minimal", "uart"),
    ("bleprph_minimal", "spi_fd"),
    ("bleprph_minimal", "spi_hd"),
    ("bleprph_gatt", "sdio"),
]


@pytest.mark.sanity
@pytest.mark.parametrize("example,transport", BT_CELLS)
def test_bt_nimble_advertising(bench, example, transport):
    """NimBLE host reaches advertising over the hosted transport."""
    b = bench(f"bluetooth/nimble_hosted_hci/{example}", "mcu_host", transport,
              timeout="180s")
    r = eh_test_expect(b["host"], ADVERTISING, fail=FATAL_PATTERNS, timeout=150)
    assert r.ok, f"{example} did not reach 'advertising as' on {transport}"
