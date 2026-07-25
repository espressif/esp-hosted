# SPDX-License-Identifier: Apache-2.0
"""Hosted Bluetooth on the emulator (NimBLE + Bluedroid, BLE only).

The host (P4) runs the BT host stack; the CP (C6) is a controller-only
`hosted_hci` bridge that relays HCI over the transport. Reaching each example's
"ready" line proves the full hosted HCI round-trip completed (host stack →
transport → CP controller → event back) — the controller-works signal.

Depth here is init/advertising via esp-emu's built-in virtual controller (no
external peer). Connect/scan/GATT (ACL data path) needs Bumble + bench wiring —
tracked separately. Classic BR/EDR is out of scope on the P4-C6 emu.
"""

import pytest

from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

# (example, transport, ready_regex). bleprph_minimal proves every wire; the rest
# prove each scenario/stack over the (already transport-proven) SDIO path.
BT_CELLS = [
    # NimBLE (hosted VHCI)
    ("nimble_hosted_hci/bleprph_minimal", "sdio",   r'advertising as '),
    ("nimble_hosted_hci/bleprph_minimal", "uart",   r'advertising as '),
    ("nimble_hosted_hci/bleprph_minimal", "spi_fd", r'advertising as '),
    ("nimble_hosted_hci/bleprph_minimal", "spi_hd", r'advertising as '),
    ("nimble_hosted_hci/bleprph_gatt",    "sdio",   r'advertising as '),
    ("nimble_hosted_hci/bleprph_wifi_coex", "sdio", r'BLE Host Task Started'),
    # NOTE: Bluedroid examples are NOT listed. They reach the hosted bridge, but
    # bluedroid's strict HCI parser asserts on the emu built-in controller's short
    # Command-Complete for LE init queries the emu doesn't implement (0x201C fixed,
    # 0x2024 next, more likely — even controller_mac_addr crashes after the read).
    # Completing the emu's LE HCI command set is the follow-up; add cells once it
    # lands. NimBLE tolerates the gaps, so it is fully covered here.
]


@pytest.mark.sanity
@pytest.mark.parametrize("example,transport,ready", BT_CELLS,
                         ids=[f"{e.split('/')[-1]}-{t}" for e, t, _ in BT_CELLS])
def test_bt_hosted(bench, example, transport, ready):
    """Hosted BT example reaches its ready line over the transport."""
    b = bench(f"bluetooth/{example}", "mcu_host", transport, timeout="180s")
    r = eh_test_expect(b["host"], ready, fail=FATAL_PATTERNS, timeout=150)
    assert r.ok, f"{example} did not reach {ready!r} on {transport}"
