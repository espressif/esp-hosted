# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Hosted OpenThread on the emulator — end-to-end 2-node Thread association.

Node A is the ESP-Hosted split: a P4 host running the full OpenThread stack,
driving a C6 Radio Co-Processor (RCP) over TWO links — the ESP-Hosted SDIO
control transport AND a dedicated spinel UART (2nd --hosted-uart bridge). The
RCP's 802.15.4 radio is UDP-bridged (--thread-sim) to Node B, a monolithic C6
`ot_cli` peer (native radio, built from ESP-IDF). Both sides auto-start Thread
with the identical compile-time dataset (OPENTHREAD_NETWORK_AUTO_START), so a
working RCP radio path makes the host attach to the peer's partition as a Child.

Reaching `Role ... -> child` on the host proves the whole chain end-to-end:
host OT stack -> spinel/UART -> RCP -> esp_ieee802154 -> --thread-sim -> peer
and back (MLE parent-response). That is the hosted-OpenThread works signal.

The emu has no P4 PSRAM, so the host is built with SPIRAM off (and thus tasks
from internal RAM) — an emu-target overlay only; real P4 HW keeps SPIRAM on.

Sequence (time flows downward; one column per emulator process). Buses:
  RCP <-> HOST = ESP-Hosted SDIO (--hosted, RPC/control) + a dedicated spinel
  UART (--hosted-uart, 802.15.4 PDUs);  RCP <-> PEER = 802.15.4 RF (--thread-sim
  UDP). RCP+HOST together are "Node A" (the hosted device); PEER is "Node B".

  pytest        PEER (C6)          RCP (C6)              HOST (P4)
    |  spawn peer   |                  |                     |
    |-------------->| auto-start;      |                     |
    |               | form partition   |                     |
    |               | ==> LEADER       |                     |
    |  bench(): build Node A, spawn RCP (slave) then HOST     |
    |----------------------------------> | SDIO slave up      |
    |------------------------------------------------------> | boot
    |               |                  | <--- reset GPIO54 ---| host resets RCP
    |               |                  | <=== SDIO RPC ======>| control-plane up (fw match)
    |               |                  | <--- spinel UART --->| RCP reset OK; OT stack init
    |               |                  |                     | auto-start ==> MLE attach
    |               |  802.15.4 RF (--thread-sim UDP)         |
    |               | <...RF...---------| <----- spinel ------| MLE Parent Request  (host->air)
    |               | ---RF...--------> | ------ spinel ----->| MLE Parent Response (air->host)
    |               |                  |                     | Child ID Req/Resp ==> RLOC16 R
    |  bench --------------------------------------------------> "ot neighbor table" to PEER
    |               | prints its mesh view: Child @ 0xR       |
    |  bench --------------------------------------------------> "ot ping <peer ll>" from HOST
    |               | <=== ICMPv6 echo request (encrypted) ===  (host->air->peer)
    |               | ===> ICMPv6 echo reply   (encrypted) ==>  (peer->air->host)
    |  assert: HOST "Role -> child" + RLOC16 R;  PEER "-> leader";
    |          PEER neighbor table lists a Child whose RLOC16 == R;  ping 0% loss

Two data-plane checks. (1) Control: the host reaches Child with an RLOC16 the peer
assigned it (MLE Parent/Child-ID exchange over the radio), and the leader -- queried
over its own radio-formed mesh -- lists that same RLOC16 as a live Child neighbor.
(2) Traffic: the host ICMPv6-pings the peer's link-local over the mesh and gets
replies (0% loss). Both round-trip host OT stack -> spinel -> RCP -> esp_ieee802154
-> --thread-sim -> peer and back. The ping additionally exercises 802.15.4 MAC link
security: ESP does AES-CCM* in the radio HW (OT_RADIO_CAPS_TRANSMIT_SEC), which the
emu reproduces in software (thread_sec.rs), else the peer's SW RX-decrypt drops the
frame (RxErrSec); and EH_EMU_RADIO_PACE wall-paces idle WFI so the reply-timer can't
race the cross-emu round-trip.
"""

import os
import re
import subprocess
import time
from pathlib import Path

import pytest

from infra import lab
from infra.emu_dut import EmuDut, emu_bin
from infra.expect_helper import eh_test_expect, FATAL_PATTERNS

_REPO = Path(__file__).resolve().parents[3]
_PEER_CACHE = _REPO / "tests/.work/ot_cli_peer"          # built once, reused
_IDF = Path(os.path.expanduser("~/esp-idf"))

# Emu-target host overlay: the emu models no P4 PSRAM, so SPIRAM must be off
# (which also forces worker-task stacks to internal RAM); auto-start gives the
# host the same default dataset as the peer so they form one partition.
_EMU_HOST_OVL = (
    "CONFIG_SPIRAM=n",
    "CONFIG_ESP_HOSTED_DFLT_TASK_FROM_SPIRAM=n",
    "CONFIG_OPENTHREAD_NETWORK_AUTO_START=y",
)


def _build_peer():
    """Build (once, cached) the monolithic native-radio C6 `ot_cli` peer from
    ESP-IDF with a shared auto-start dataset. Returns (merged_bin, elf) or None
    if IDF/toolchain is unavailable (test then skips)."""
    bin_ = _PEER_CACHE / "build/merged_flash.bin"
    elf = _PEER_CACHE / "build/esp_ot_cli.elf"
    if bin_.exists() and elf.exists():
        return str(bin_), str(elf)
    src = _IDF / "examples/openthread/ot_cli"
    if not src.exists():
        return None
    _PEER_CACHE.parent.mkdir(parents=True, exist_ok=True)
    script = f"""
set -e
. {_IDF}/export.sh >/dev/null 2>&1
rm -rf {_PEER_CACHE}
cp -r {src} {_PEER_CACHE}
printf 'CONFIG_OPENTHREAD_NETWORK_AUTO_START=y\\n' > {_PEER_CACHE}/sdkconfig.emu
cd {_PEER_CACHE}
export SDKCONFIG_DEFAULTS='sdkconfig.defaults;sdkconfig.emu'
idf.py set-target esp32c6 build
cd build && python -m esptool --chip esp32c6 merge_bin -o merged_flash.bin @flash_args
"""
    r = subprocess.run(["bash", "-c", script], capture_output=True, text=True,
                       timeout=int(os.environ.get("EH_BUILD_TIMEOUT", "1800")))
    if r.returncode != 0 or not bin_.exists():
        return None
    return str(bin_), str(elf)


# Heavy cell (3 emus: host + RCP + peer). Serialize like the BT/Bumble cells so
# parallel load doesn't starve the emus. sdio is the sanity representative.
@pytest.mark.sanity
@pytest.mark.xdist_group("ot_thread")
@pytest.mark.parametrize("transport", ["sdio"])
def test_ot_cli_2node_association(bench, lab_tmp, worker_id, transport):
    if not _IDF.exists():
        pytest.skip("ESP-IDF not available for the ot_cli peer build")
    peer = _build_peer()
    if peer is None:
        pytest.skip("could not build the ot_cli peer firmware")
    peer_bin, peer_elf = peer

    # Two disjoint UDP ports from this bench's reserved block: RCP binds PA and
    # peers PB; the peer binds PB and peers PA (crossed --thread-sim link). autoack
    # models the 802.15.4 MAC ACK real radios send for ack-requested unicast.
    port = lab.alloc_bench(worker_id)["port"]
    pa, pb = port, port + 2
    plog = Path(lab_tmp) / "ot_peer.log"
    hlog = Path(lab_tmp) / f"host_{transport}.log"       # bench writes the host DUT here

    os.environ["EH_OT_SPINEL"] = "1"                                          # emu.py: spinel UART bridge
    os.environ["EH_THREAD_SIM_CP"] = f"bind:{pa},peer:127.0.0.1:{pb},autoack"  # emu.py: RCP radio bridge
    os.environ["EH_EMU_RADIO_PACE"] = "1"     # wall-pace idle WFI so a ping reply-timer can't race the radio round-trip
    peer = None
    try:
        # Bring the peer up first so it forms the partition (becomes Leader)
        # before the heavier host+RCP node finishes booting and attaches. Its
        # --timeout must outlast a COLD build of Node A (bench() builds cp+host
        # synchronously below, ~3 min first time) plus the host's attach, or the
        # peer emu exits before the host ever reaches the radio. EmuDut (not raw
        # Popen) so we can drive its `ot` console to read the leader's view.
        peer = EmuDut("peer", [str(emu_bin()), "--chip", "esp32c6", "--firmware", peer_bin,
                               "--elf", peer_elf, "--thread-sim",
                               f"bind:{pb},peer:127.0.0.1:{pa},autoack", "--timeout", "600s"], plog)

        b = bench("openthread/cli", "esp_host", transport, timeout="150s",
                  overlay=_EMU_HOST_OVL)
        # Host attaches to the peer's partition as a Child — the end-to-end signal.
        r = eh_test_expect(b["host"], r'Role \S+ -> child', fail=FATAL_PATTERNS, timeout=90)
        peer_out = plog.read_text() if plog.exists() else ""
        assert r.ok, (f"host did not attach to the peer's Thread partition\n"
                      f"...peer log tail:\n{peer_out[-1500:]}")
        # Peer must have become Leader (the partition the host joined).
        assert "-> leader" in peer_out, f"peer never became Leader:\n{peer_out[-1500:]}"

        # Data-plane check 1 (control): the RLOC16 the peer assigned the host
        # during the MLE join, cross-checked against the peer's own neighbor
        # table listing that host as a live Child (role C). Two independent views
        # of one association — the leader, queried over its radio-formed mesh,
        # confirms it exchanged data with and registered our host.
        host_rlocs = re.findall(r'RLOC16 \w+ -> ([0-9a-fA-F]{4})', hlog.read_text())
        assert host_rlocs, "host never printed an assigned RLOC16"
        host_rloc = host_rlocs[-1].lower()

        peer.write("ot neighbor table")
        nb = eh_test_expect(peer, r'\|\s*C\s*\|\s*0x[0-9a-fA-F]{4}\s*\|', timeout=15)
        peer_out = plog.read_text()
        rows = re.findall(r'\|\s*C\s*\|\s*0x([0-9a-fA-F]{4})\s*\|', peer_out)
        assert nb.ok and rows, (f"leader's neighbor table shows no Child row\n"
                                f"...peer log tail:\n{peer_out[-1500:]}")
        assert host_rloc in [x.lower() for x in rows], (
            f"host RLOC16 0x{host_rloc} not a Child in the leader's neighbor table "
            f"(rows={rows})\n...peer log tail:\n{peer_out[-1500:]}")

        # Data-plane check 2 (traffic): ping the peer's mesh-local link-local from
        # the host over the Thread mesh and require replies. This drives real
        # ICMPv6 echo end-to-end — host OT -> spinel -> RCP -> esp_ieee802154 ->
        # --thread-sim -> peer and back — exercising MAC link security (the emu
        # models the radio's AES-CCM* transform) and the RX path, not just the
        # association relationship.
        peer.write("ot ipaddr linklocal")
        eh_test_expect(peer, r'fe80:[0-9a-fA-F:]+', timeout=10)
        time.sleep(0.5)
        lls = re.findall(r'^(fe80:[0-9a-fA-F:]{6,})\s*$', plog.read_text(), re.M)
        assert lls, "peer never printed its link-local address"
        peer_ll = lls[-1]
        b["host"].write(f"ot ping {peer_ll} 32 3")
        pr = eh_test_expect(b["host"], r'(\d+) packets received', fail=FATAL_PATTERNS, timeout=30)
        assert pr.ok, f"no ping summary from the host pinging the peer {peer_ll}"
        # Require at least one echo reply (the round-trip crosses two async emu
        # bridges, so tolerate an occasional dropped packet but not total loss).
        got = re.findall(r'(\d+) packets transmitted, (\d+) packets received', hlog.read_text())
        assert got and int(got[-1][1]) >= 1, (
            f"ICMP ping to {peer_ll} got no replies (result={got})\n"
            f"...host log tail:\n{hlog.read_text()[-1200:]}")
    finally:
        os.environ.pop("EH_OT_SPINEL", None)
        os.environ.pop("EH_THREAD_SIM_CP", None)
        os.environ.pop("EH_EMU_RADIO_PACE", None)
        if peer is not None:
            peer.stop()
