# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
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

import contextlib
import os
import subprocess
import time
from pathlib import Path

import pytest

from infra import lab
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

_REPO = Path(__file__).resolve().parents[3]
_BUMBLE = _REPO / ".deps/esp-emu/tools/bumble_test.py"
_VENV_PY = _REPO / ".deps/testvenv/bin/python"


def _wait_listening(logpath, timeout=60):
    """Block until Bumble logs that its HCI TCP server is listening. The CP's HCI
    connect is single-shot, so starting the emu on a fixed sleep races the bind
    under load -> 'Connection refused' -> built-in fallback -> Bumble sees no advert."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if logpath.exists() and "HCI controller listening" in logpath.read_text():
            return True
        time.sleep(0.1)
    return False

# (example, transport, ready_regex). bleprph_minimal proves every wire; the rest
# prove each scenario/stack over the (already transport-proven) SDIO path.
BT_CELLS = [
    # NimBLE (hosted VHCI)
    ("esp_hosted_nimble/bleprph_minimal", "sdio",   r'advertising as '),
    ("esp_hosted_nimble/bleprph_minimal", "uart",   r'advertising as '),
    ("esp_hosted_nimble/bleprph_minimal", "spi_fd", r'advertising as '),
    ("esp_hosted_nimble/bleprph_minimal", "spi_hd", r'advertising as '),
    ("esp_hosted_nimble/bleprph_gatt",    "sdio",   r'advertising as '),
    ("esp_hosted_nimble/bleprph_wifi_coex", "sdio", r'BLE Host Task Started'),
    # NOTE: Bluedroid is not in these BUILT-IN-controller advertising cells: its
    # strict HCI parser asserts on the emu built-in controller's short Command-
    # Complete for LE init queries the emu doesn't implement (0x201C fixed, 0x2024
    # next...). NimBLE tolerates the gaps. Bluedroid IS covered end-to-end via
    # test_bt_central_connect below (Bumble is a complete controller). Completing
    # the emu built-in LE set to also cover bluedroid built-in advertising is a
    # follow-up.
]


@pytest.mark.sanity
@pytest.mark.parametrize("example,transport,ready", BT_CELLS,
                         ids=[f"{e.split('/')[-1]}-{t}" for e, t, _ in BT_CELLS])
def test_bt_hosted(bench, example, transport, ready):
    """Hosted BT example reaches its ready line over the transport."""
    b = bench(f"bluetooth/{example}", "mcu_host", transport, timeout="180s")
    r = eh_test_expect(b["host"], ready, fail=FATAL_PATTERNS, timeout=150)
    assert r.ok, f"{example} did not reach {ready!r} on {transport}"


def _cc(example, transport, ready=r'advertising as ', sanity=False):
    return pytest.param(example, transport, ready,
                        id=f"{example.split('/')[-1]}-{transport}",
                        marks=[pytest.mark.sanity] if sanity else [])


@contextlib.contextmanager
def _bumble_bench(bench, lab_tmp, worker_id, example, transport, ready, wait_bumble=True):
    """Run `example` (mcu_host) under a Bumble virtual central: Bumble scans+connects
    over the hosted transport while the host advertises. Yields (bench, bumble_output)
    once the host has reached `ready` and Bumble has finished (or timed out).

    Bumble is a *complete* controller substitute, so it answers the LE-init queries
    the emu built-in controller lacks — this is what lets Bluedroid work here (vs.
    the built-in-controller advertising path in test_bt_hosted). The CP forwards its
    HCI to Bumble via --ble-hci (EH_BLE_HCI_PORT)."""
    # Environment gate, NOT a masked failure: a checkout without the emu bench
    # (esp-emu clone + Bumble in the venv) genuinely cannot run this cell — a
    # new-user / build-only environment differs from the lab box, so skipping is
    # correct here. ALL-GREEN forbids hiding a real failure, not skipping when the
    # optional bench is absent (cf. the OT peer-source and nw_split AP skips).
    if not _VENV_PY.exists() or not _BUMBLE.exists():
        pytest.skip("bumble / esp-emu tooling not available")
    port = lab.alloc_bench(worker_id)["port"]  # reserved per-worker range, not OS-ephemeral
    blog = Path(lab_tmp) / "bumble.log"
    os.environ["EH_BLE_HCI_PORT"] = str(port)
    bp_box = []  # holds the Bumble proc; a list so the pre_launch closure can set it

    def _start_bumble():
        # Run by bench() AFTER the firmware is built, BEFORE the emus launch, so
        # Bumble's connect deadline covers only emu bring-up. Starting it before the
        # build (as this test used to) let a cold-cache build (minutes) outlast the
        # deadline -> Bumble exits -> closes the HCI socket -> CP --ble-hci hits
        # ECONNREFUSED and the whole cell fails. (bf closes here; the child keeps its
        # dup'd fd, matching the previous behaviour.)
        with open(blog, "w") as bf:
            bp_box.append(subprocess.Popen([str(_VENV_PY), "-u", str(_BUMBLE)], env=os.environ,
                                           stdout=bf, stderr=subprocess.STDOUT))
        if not _wait_listening(blog):
            pytest.fail("Bumble HCI controller never started listening")

    try:
        b = bench(f"bluetooth/{example}", "mcu_host", transport, timeout="240s",
                  pre_launch=_start_bumble)
        r = eh_test_expect(b["host"], ready, fail=FATAL_PATTERNS, timeout=90)
        assert r.ok, f"{example} did not reach {ready!r} on {transport}"
        if bp_box and wait_bumble:
            try:
                bp_box[0].wait(timeout=180)
            except subprocess.TimeoutExpired:
                pass
        yield b, (blog.read_text() if blog.exists() else "")
    finally:
        os.environ.pop("EH_BLE_HCI_PORT", None)
        bp = bp_box[0] if bp_box else None
        if bp and bp.poll() is None:
            bp.terminate()
            try:
                bp.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bp.kill()


# End-to-end connect over every transport. sdio is the @sanity representative
# per scenario; uart/spi_fd/spi_hd run under --regression. Serialized onto one
# xdist worker (xdist_group): each cell is heavy (Bumble + two emus), so letting
# them pile up under parallel load starves the emu and Bumble misses the advert.
@pytest.mark.xdist_group("bt_bumble")
@pytest.mark.parametrize("example,transport,ready", [
    _cc("esp_hosted_nimble/bleprph_gatt", "sdio", sanity=True),
    _cc("esp_hosted_nimble/bleprph_gatt", "uart"),
    _cc("esp_hosted_nimble/bleprph_gatt", "spi_fd"),
    _cc("esp_hosted_nimble/bleprph_gatt", "spi_hd"),
    _cc("esp_hosted_bluedroid/ble_gatt_server", "sdio", sanity=True),
    _cc("esp_hosted_bluedroid/ble_gatt_server", "uart"),
    _cc("esp_hosted_bluedroid/ble_gatt_server", "spi_fd"),
    _cc("esp_hosted_bluedroid/ble_gatt_server", "spi_hd"),
    # Second Bluedroid GATTS example (full compatibility-test attribute table),
    # proving the connect path beyond ble_gatt_server. Its advertise log differs.
    _cc("esp_hosted_bluedroid/ble_compatibility_test", "sdio",
        ready=r'advertising start successfully'),
])
def test_bt_central_connect(bench, lab_tmp, worker_id, example, transport, ready):
    """Bumble (virtual central) scans, finds, and CONNECTS to the host peripheral
    over the hosted transport, then discovers its first service — the ACL data
    path proven end to end (advertise -> scan -> connect).

    Runs both host stacks: NimBLE and Bluedroid.

    Full GATT characteristic read/write still hits an ACL round-trip timeout
    (ATT_READ_BY_TYPE) over the hosted path — tracked as a follow-up."""
    with _bumble_bench(bench, lab_tmp, worker_id, example, transport, ready) as (b, out):
        assert "[+] Connected!" in out, f"bumble did not connect to the peripheral:\n...{out[-2000:]}"


@pytest.mark.sanity
@pytest.mark.xdist_group("bt_bumble")
def test_bt_set_mac(bench, lab_tmp, worker_id):
    """Setting the BT controller MAC takes effect on the co-processor controller.

    controller_mac_addr reads the controller's BD_ADDR, sets a known one
    (08:3a:8d:01:01:01) via esp_hosted_iface_mac_addr_set(ESP_MAC_BT), then reads
    it back. The read is a live round-trip to the CP: it returns the CP's real
    factory address *before* the set (observed 24:0a:c4:00:00:03) and the
    configured address *after* — so reaching the post-set line proves the CP
    controller accepted the new MAC, not merely that the host echoed it back.

    Runs under Bumble as the controller: Bluedroid can't drive the emu built-in
    controller (its strict HCI parser asserts — see test_bt_hosted), so Bumble
    stands in so the app runs cleanly. The MAC proof is captured at boot, before
    the stack advertises; we then wait for advertising to confirm a clean bring-up
    before teardown.

    On-air verification (a scanning central seeing the configured BD_ADDR) is not
    reachable on the emu: in the Bumble path Bumble *is* the radio, so a CP-set MAC
    never reaches it. Tracked as a follow-up."""
    with _bumble_bench(bench, lab_tmp, worker_id,
                       "esp_hosted_bluedroid/controller_mac_addr", "sdio",
                       ready=r'New BT controller mac address is 08:3a:8d:01:01:01',
                       wait_bumble=False) as (b, _out):
        # gate above already asserted the round-trip; let Bluedroid finish coming
        # up under Bumble so teardown is graceful (no mid-bring-up controller loss).
        r = eh_test_expect(b["host"], r'Advertising start successfully',
                           fail=FATAL_PATTERNS, timeout=30)
        assert r.ok, "Bluedroid did not finish bring-up under Bumble after the MAC set"
