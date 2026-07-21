"""The Target contract — the substrate seam.

`BenchProvider.make(spec) -> Bench`. `spec` names *what* to run (example, roles,
targets, transport, wake, timeout); the provider knows *how* to provision it on
its substrate. `Bench` exposes the pytest-embedded `dut` shape (`duts=[host, cp]`)
plus an optional `net` stimulus and a `down()` teardown, so every test body reads
identically regardless of substrate.
"""
from dataclasses import dataclass, field

# Capability tags — a provider detects its own properties at runtime and stamps
# them on the Bench; test bodies query these instead of branching on substrate
# name. A new substrate or board changes what it *reports*, not the test corpus.
CAP_WAKE = "wake"              # host power-save wake supported (wake GPIO / net stimulus)
CAP_NET = "net"               # a Net packet-stimulus is attached to this bench
CAP_COEX_WIRED = "coex_wired" # external-coex physically wired (config ops return rc=0)
CAP_GPIO_LOOPBACK = "gpio_loopback"  # GPIO reads back what was driven (emu models it;
                                     # real HW needs a physical loopback wire)
CAP_WIFI_ITWT = "wifi_itwt"   # coprocessor supports 802.11ax iTWT (HE SoC: C5/C6/C61)


@dataclass
class BenchSpec:
    """What to run — substrate-agnostic. Providers ignore fields that don't apply."""
    example: str
    host_role: str = "mcu_host"      # mcu_host | esp_host | linux_802_3_host/c_app
    cp_role: str = "cp"
    host_target: str = "esp32p4"
    cp_target: str = "esp32c6"
    transport: str = "sdio"          # sdio | uart
    wake: bool = False               # provide a Net stimulus + host-forward port
    timeout: str = "60s"             # per-DUT run cap (EH_EMU_TIMEOUT overrides)
    extra_ovl: tuple = ()            # per-test host sdkconfig lines (e.g. wifi creds)
    cp_extra_ovl: tuple = ()         # per-test CP sdkconfig lines (e.g. SDIO SW_AGGR)
    hostfwd: tuple = ()              # extra CP guest ports to forward for network-split
                                     # routing tests (e.g. 61500 echo, 61600 http);
                                     # each maps to a distinct host loopback port,
                                     # reachable via net.request(guest_port, ...)
    cp_app_to_host: str = None       # stage the built CP app .bin into the host build
                                     # at this rel path (LittleFS OTA image source)
    cp_ota_ovl: tuple = ()           # extra CP sdkconfig lines for the STAGED OTA
                                     # image only — lets a test OTA the CP into a
                                     # different config than it booted with (e.g.
                                     # stream -> SW_AGGR migration drill)


@dataclass
class Bench:
    """A provisioned bench. `duts` is [host, cp] (pytest-embedded shape); `net` is
    an optional stimulus provider; `down` is the teardown the provider installed;
    `caps` is the runtime-detected capability set (CAP_* tags) the test body reads."""
    duts: list
    net: object = None
    teardown: object = None          # callable() or None
    caps: frozenset = field(default_factory=frozenset)

    @property
    def host(self):
        return self.duts[0]

    @property
    def cp(self):
        return self.duts[1] if len(self.duts) > 1 else None

    def down(self):
        if self.teardown:
            self.teardown()


class BenchProvider:
    """One per substrate. `make` provisions a Bench (build → up); the returned
    Bench's `down()` releases it. Implementations: targets.emu.EmuTarget,
    targets.linux.LinuxTarget, targets.serial.SerialTarget (HW).

    `transports` = the wires this substrate can provide; the bench factory skips a
    test that asks for one it can't. `caps` = substrate-level feature capabilities
    (CAP_* tags) merged into every Bench it makes. Both are runtime properties of
    the substrate/bench, not hardcoded per test."""

    transports = frozenset({"sdio", "uart"})
    caps = frozenset()

    def make(self, spec: BenchSpec, *, worker_id: str, lab_tmp) -> Bench:
        raise NotImplementedError
