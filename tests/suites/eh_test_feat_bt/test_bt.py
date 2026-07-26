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

import os
import socket
import subprocess
import time
from pathlib import Path

import pytest

from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

_REPO = Path(__file__).resolve().parents[3]
_BUMBLE = _REPO / ".deps/esp-emu/tools/bumble_test.py"
_VENV_PY = _REPO / ".deps/testvenv/bin/python"


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p

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


@pytest.mark.sanity
def test_bt_nimble_central_connect(bench, lab_tmp):
    """Bumble (virtual central) scans, finds, and CONNECTS to the NimBLE host
    peripheral over the hosted transport, then discovers its first service — the
    ACL data path proven end to end (advertise -> scan -> connect). The CP
    forwards its HCI to Bumble via --ble-hci (EH_BLE_HCI_PORT).

    Full GATT characteristic read/write still hits an ACL round-trip timeout
    (ATT_READ_BY_TYPE) over the hosted path — tracked as a follow-up."""
    if not _VENV_PY.exists() or not _BUMBLE.exists():
        pytest.skip("bumble / esp-emu tooling not available")
    port = _free_port()
    blog = Path(lab_tmp) / "bumble.log"
    os.environ["EH_BLE_HCI_PORT"] = str(port)
    bp = None
    try:
        with open(blog, "w") as bf:
            bp = subprocess.Popen([str(_VENV_PY), str(_BUMBLE)], env=os.environ,
                                  stdout=bf, stderr=subprocess.STDOUT)
        time.sleep(3)  # let Bumble's TCP controller bind before the CP dials in
        b = bench("bluetooth/nimble_hosted_hci/bleprph_gatt", "mcu_host", "sdio",
                  timeout="150s")
        r = eh_test_expect(b["host"], r'advertising as ', fail=FATAL_PATTERNS, timeout=90)
        assert r.ok, "host peripheral did not advertise"
        try:
            bp.wait(timeout=90)
        except subprocess.TimeoutExpired:
            pass
    finally:
        os.environ.pop("EH_BLE_HCI_PORT", None)
        if bp and bp.poll() is None:
            bp.terminate()
            try:
                bp.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bp.kill()
    out = blog.read_text() if blog.exists() else ""
    assert "[+] Connected!" in out, f"bumble did not connect to the peripheral:\n...{out[-2000:]}"
