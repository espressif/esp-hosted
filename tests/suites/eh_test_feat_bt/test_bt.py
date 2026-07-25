# SPDX-License-Identifier: Apache-2.0
"""Hosted Bluetooth (NimBLE over ESP-Hosted) on the emulator.

The host (P4) runs the NimBLE stack; the CP (C6) is a controller-only
`hosted_hci` bridge that relays HCI over the transport. Success = the host
reaches the advertising state, which only happens after the full controller
init round-trip completes — so it is the "controller works" signal.

Advertising is reachable with esp-emu's built-in virtual BLE controller (no
external peer / --ble-hci). Connect + GATT would additionally need Bumble.
"""

import pytest

from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

BT_EXAMPLE = "bluetooth/nimble_hosted_hci/bleprph_minimal"
ADVERTISING = r'advertising as '


@pytest.mark.sanity
def test_bt_nimble_advertising_sdio(bench):
    """NimBLE host reaches advertising over the SDIO hosted transport."""
    b = bench(BT_EXAMPLE, "mcu_host", "sdio", timeout="150s")
    r = eh_test_expect(b["host"], ADVERTISING, fail=FATAL_PATTERNS, timeout=120)
    assert r.ok, "host did not reach 'advertising as'"
