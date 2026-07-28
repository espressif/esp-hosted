# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Hosted Zigbee on the emulator — coordinator network formation.

Node A is the ESP-Hosted split: a P4 host running the esp-zigbee thermostat
COORDINATOR, driving a C6 Radio Co-Processor over TWO links — the ESP-Hosted
SDIO control transport AND a dedicated spinel UART. The RCP is the SAME firmware
as the OpenThread RCP: the 802.15.4 radio is stack-agnostic, so Zigbee and Thread
share it byte-for-byte.

On boot the coordinator drives the RCP through an 802.15.4 energy scan (over the
spinel/radio path), selects a channel, forms a PAN, and opens it for joining.
Reaching "Formed network successfully" + "Network(...) is open" proves the whole
hosted-Zigbee bring-up end to end: host esp-zigbee stack -> spinel -> RCP ->
esp_ieee802154 energy-detect -> channel select -> PAN form -> beacon.

  pytest        RCP (C6)              HOST (P4, coordinator)
    |  bench(): build Node A, spawn RCP (slave) then HOST      |
    |----------------------------------> | SDIO slave up        |
    |------------------------------------------------------->  | boot esp-zigbee
    |               | <=== SDIO RPC ======>| control-plane up (fw match)
    |               | <--- spinel UART --->| RCP reset OK; radio init
    |               | <-- energy scan ----  | ED over the RCP radio
    |               |                      | pick channel; FORM PAN; open
    |  assert: HOST "Formed network successfully" AND "Network(..) is open"

Full device JOIN (a separate joiner associating into this network) is NOT
emulated: it needs 802.15.4 active-scan / beacon / association MAC timing that
the raw-PHY --thread-sim bridge does not model — the joiner's channel-hopping
scan and the beacon round-trip cannot be aligned in the emulator's discrete
channel-hop + WFI-injection model (verified: a HA_on_off_light joiner's network
steering fails even with radio wall-pacing + auto-ACK + CCM* link security). Full
join is covered on real 3-chip hardware; the emulator faithfully models network
formation, which is what this test asserts.
"""

import os

import pytest

from infra import lab
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

# The emu models no P4 PSRAM; the committed Zigbee example already builds without
# SPIRAM (fits internal RAM), so no host overlay is needed beyond the bench's own
# FUNC_BOARD default.


# Heavy cell (host + RCP over spinel). Join the emu_heavy loadgroup so it
# serializes with the other heavy cells instead of oversubscribing the box.
@pytest.mark.sanity
@pytest.mark.xdist_group("emu_heavy")
@pytest.mark.parametrize("transport", ["sdio"])
def test_zigbee_coordinator_forms(bench, worker_id, transport):
    # A --thread-sim radio bind is needed so the RCP's 802.15.4 energy scan
    # completes; no peer is required (formation is unilateral). autoack + radio
    # pacing match the OpenThread cell's radio model.
    port = lab.alloc_bench(worker_id)["port"]
    pa, pb = port, port + 2
    os.environ["EH_OT_SPINEL"] = "1"                                          # emu.py: spinel UART bridge
    os.environ["EH_THREAD_SIM_CP"] = f"bind:{pa},peer:127.0.0.1:{pb},autoack"  # emu.py: RCP radio bridge
    os.environ["EH_EMU_RADIO_PACE"] = "1"
    try:
        b = bench("zigbee/thermostat", "esp_host", transport, timeout="150s")
        # Coordinator forms a PAN (energy scan -> channel -> form) over the RCP.
        r = eh_test_expect(b["host"], r'Formed network successfully', fail=FATAL_PATTERNS, timeout=90)
        assert r.ok, "hosted Zigbee coordinator did not form a network over the RCP"
        # ...and opens it for joining (the end-to-end formation signal).
        r2 = eh_test_expect(b["host"], r'Network\(0x[0-9a-fA-F]+\) is open', fail=FATAL_PATTERNS, timeout=30)
        assert r2.ok, "coordinator formed but never opened the network for joining"
    finally:
        os.environ.pop("EH_OT_SPINEL", None)
        os.environ.pop("EH_THREAD_SIM_CP", None)
        os.environ.pop("EH_EMU_RADIO_PACE", None)
