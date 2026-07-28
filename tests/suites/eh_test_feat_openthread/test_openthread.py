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
    |               |                  |                     | RLOC16 = parent+1 ==> CHILD
    |  assert: HOST log "Role ... -> child"  AND  PEER log "-> leader"

The child/leader + RLOC16=parent+1 is the end-to-end proof: a radio TX issued by
the host's OT stack traversed spinel->RCP->esp_ieee802154->--thread-sim to the
peer and the MLE response came all the way back.
"""

import os
import subprocess
import time
from pathlib import Path

import pytest

from infra import lab
from infra.emu_dut import emu_bin
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
    # peers PB; the peer binds PB and peers PA (crossed --thread-sim link).
    port = lab.alloc_bench(worker_id)["port"]
    pa, pb = port, port + 2
    plog = Path(lab_tmp) / "ot_peer.log"

    os.environ["EH_OT_SPINEL"] = "1"                                  # emu.py: spinel UART bridge
    os.environ["EH_THREAD_SIM_CP"] = f"bind:{pa},peer:127.0.0.1:{pb}"  # emu.py: RCP radio bridge
    peer_proc = None
    try:
        # Bring the peer up first so it forms the partition (becomes Leader)
        # before the heavier host+RCP node finishes booting and attaches. Its
        # --timeout must outlast a COLD build of Node A (bench() builds cp+host
        # synchronously below, ~3 min first time) plus the host's attach, or the
        # peer emu exits before the host ever reaches the radio.
        with open(plog, "w") as pf:
            peer_proc = subprocess.Popen(
                [str(emu_bin()), "--chip", "esp32c6", "--firmware", peer_bin,
                 "--elf", peer_elf, "--thread-sim", f"bind:{pb},peer:127.0.0.1:{pa}",
                 "--timeout", "600s"], stdout=pf, stderr=subprocess.STDOUT)

        b = bench("openthread/cli", "esp_host", transport, timeout="150s",
                  overlay=_EMU_HOST_OVL)
        # Host attaches to the peer's partition as a Child — the end-to-end signal.
        r = eh_test_expect(b["host"], r'Role \S+ -> child', fail=FATAL_PATTERNS, timeout=90)
        peer_out = plog.read_text() if plog.exists() else ""
        assert r.ok, (f"host did not attach to the peer's Thread partition\n"
                      f"...peer log tail:\n{peer_out[-1500:]}")
        # Peer must have become Leader (the partition the host joined).
        assert "-> leader" in peer_out, f"peer never became Leader:\n{peer_out[-1500:]}"
    finally:
        os.environ.pop("EH_OT_SPINEL", None)
        os.environ.pop("EH_THREAD_SIM_CP", None)
        if peer_proc and peer_proc.poll() is None:
            peer_proc.terminate()
            try:
                peer_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                peer_proc.kill()
                try:
                    peer_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
