"""EmuTarget — the esp-emu substrate: a C6 CP + P4 host joined by a Unix-socket
bridge (SDIO or framed-UART). Each DUT is an `EmuDut` over one `esp-emu` process.

Absorbs the launch logic that was duplicated across the emu-mcu conftest's
`_bench` (network-split/wake, sdio) and `emu_bench` (parametrized transport)
fixtures — they are now one provider, `_bench` being just a specific BenchSpec.
"""
import os
import socket
import struct
import subprocess
import time

from infra import build_fw, lab
from infra.emu_dut import EmuDut, emu_bin, settle_check

from .base import (CAP_GPIO_LOOPBACK, CAP_NET, CAP_WAKE, CAP_WIFI_ITWT,
                   Bench, BenchProvider, BenchSpec)

_WAKE_GPIO = "2"     # CP→host wake line (framed bridge carries it as OP_WAKE)
_RESET_GPIO = "54"   # host→CP reset line (OP_RESET)

# The emulator's C6 wifi model advertises a fixed WPA2-PSK SoftAP; STA-connect
# tests on the emu associate to this (real benches use lab.local.json wifi_ap).
EMU_SOFTAP = {"ssid": "myssid", "password": "mypassword"}

# ESP-Hosted SPI transport GPIOs, P4-C6 Function-EV board (both sides FUNC_BOARD).
# handshake (FD only) + data-ready are slave→host, forwarded out-of-band as bridge
# GPIO frames; the emu maps the CP's pin to the host's. Reset is _RESET_GPIO (54).
# cp_cs is the CP's CS *input*: the emu replays the master's per-transaction CS
# pulse on it so the CP's DEASSERT_HS_ON_CS edge ISR (arm-next) runs — FD stalls
# without it (the emu's default is the old board's GPIO 10, not FUNC_BOARD's 18).
_SPI_PINS = {
    "spi_fd": {"cp_hs": "22", "cp_dr": "23", "cp_cs": "18", "host_hs": "16", "host_dr": "17"},
    "spi_hd": {"cp_hs": None, "cp_dr": "2",  "cp_cs": "18", "host_hs": None, "host_dr": "6"},
}

_emu_help_cache = {}


def _emu_supports(emu, flag: str) -> bool:
    """Probe `esp-emu --help` for a flag, so SPI benches skip cleanly on an
    emu binary that predates the SPI bridge instead of failing mid-launch."""
    key = str(emu)
    if key not in _emu_help_cache:
        try:
            _emu_help_cache[key] = subprocess.run(
                [key, "--help"], capture_output=True, text=True, timeout=10
            ).stdout
        except (OSError, subprocess.TimeoutExpired):
            _emu_help_cache[key] = ""
    return flag in _emu_help_cache[key]


class Net:
    """Stimulus provider over the CP's user-net host-forward. `wake()` fires a
    fire-and-forget packet at the host-reserved port (wake-on-packet). `request()`
    opens a connection to a mapped CP guest port and returns the reply — used by
    network-split routing tests to prove which stack (host vs CP) answers a given
    port class. `fwd` maps guest port → host loopback port (see EmuTarget.make)."""

    def __init__(self, port, fwd=None):
        self._port = port
        self._fwd = fwd or {22: port}

    def _shoot(self, payload, host_port):
        try:
            s = socket.socket()
            s.settimeout(2)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
            s.connect(("127.0.0.1", host_port))
            s.sendall(payload)
            s.close()
        except OSError:
            pass

    def wake(self, payload=b"wakeup-host"):
        self._shoot(payload, self._port)

    def request(self, guest_port, payload=b"", read_bytes=512, timeout=4):
        """Connect to the CP guest `guest_port` (via its host-forward), send
        `payload`, return whatever bytes come back (b"" on refusal/timeout —
        e.g. no server on that port, or the frame was routed to the other stack
        and no listener answered). Non-throwing; the caller asserts on the reply."""
        hp = self._fwd.get(int(guest_port))
        if hp is None:
            raise KeyError(f"guest port {guest_port} not forwarded (add to BenchSpec.hostfwd)")
        try:
            s = socket.socket()
            s.settimeout(timeout)
            s.connect(("127.0.0.1", hp))
            if payload:
                s.sendall(payload)
            data = s.recv(read_bytes)
            s.close()
            return data
        except OSError:
            return b""


class EmuTarget(BenchProvider):
    # The emulator provides both wires, models the wake path (net stimulus), and
    # models CP GPIO (so GPIO reads back what was driven). It does NOT model
    # external-coex wiring, so coex config ops are drivable-only.
    transports = frozenset({"sdio", "uart", "spi_fd",
                            "spi_hd", "spi_hd_1", "spi_hd_2", "spi_hd_4"})
    caps = frozenset({CAP_WAKE, CAP_GPIO_LOOPBACK, "wifi_ap",
                      CAP_WIFI_ITWT})  # emu CP = C6 (HE)

    def __init__(self):
        pass  # per-bench resources come from lab.alloc_bench (survives EmuTarget churn)

    def make(self, spec: BenchSpec, *, worker_id, lab_tmp) -> Bench:
        emu = emu_bin()
        timeout = os.environ.get("EH_EMU_TIMEOUT", spec.timeout)  # per-phase override
        transport = spec.transport
        uart = transport == "uart"
        spi_hd = transport.startswith("spi_hd")
        spi = transport == "spi_fd" or spi_hd
        if spi:
            flag = "--hosted-spi-hd" if spi_hd else "--hosted-spi"
            if not _emu_supports(emu, flag):
                import pytest
                pytest.skip(f"esp-emu at {emu} lacks {flag} (rebuild esp-emulator)")
        if transport == "spi_fd":
            cp_ovl = ["CONFIG_EH_TRANSPORT_CP_SPI=y"]
            host_bus_ovl = ["CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI=y"]
        elif spi_hd:
            # spi_hd defaults to the Kconfig default (4-line); spi_hd_1/2/4
            # pin the data-line count on both ends.
            cp_ovl = ["CONFIG_EH_TRANSPORT_CP_SPI_HD=y"]
            host_bus_ovl = ["CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD=y"]
            lines = transport.rsplit("_", 1)[1] if transport != "spi_hd" else None
            if lines:
                suffix = "1_DATA_LINE" if lines == "1" else f"{lines}_DATA_LINES"
                cp_ovl.append(f"CONFIG_EH_TRANSPORT_CP_SPI_HD_{suffix}=y")
                host_bus_ovl.append(f"CONFIG_ESP_HOSTED_HOST_SPI_HD_{suffix}=y")
        elif uart:
            cp_ovl = ["CONFIG_EH_TRANSPORT_CP_UART=y"]
            host_bus_ovl = ["CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART=y"]
        else:  # sdio
            cp_ovl = []
            host_bus_ovl = []
        # Host also builds for the P4-C6 Function-EV board so its GPIOs (reset, SPI
        # signals) match the CP's FUNC_BOARD pinout; else the two sides disagree.
        host_board_ovl = (["CONFIG_ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD=y"]
                          if spec.host_target == "esp32p4" else [])
        host_ovl = host_board_ovl + host_bus_ovl + list(spec.extra_ovl)
        cp_bus_ovl = list(cp_ovl)              # transport-derived only
        # Base image assumes the P4-C6 Function-EV board so board-wired defaults
        # (e.g. host-wakeup GPIO) resolve; product Kconfig keeps board=NONE -> -1.
        if spec.cp_target == "esp32c6":
            cp_bus_ovl.append("CONFIG_ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD=y")
        cp_ovl = cp_bus_ovl + list(spec.cp_extra_ovl)

        cp_fw = build_fw.build(spec.example, spec.cp_role, spec.cp_target, cp_ovl)
        # Stage the freshly-built CP app image into the host build (e.g. the host's
        # LittleFS OTA source dir) — app-only .bin, not merged, so the host OTAs
        # exactly the CP app it was built against.
        host_inject = None
        if spec.cp_app_to_host:
            ota_fw = cp_fw
            if spec.cp_ota_ovl:
                # Stage an OTA image with a DIFFERENT CP config than the boot
                # image (e.g. stream -> SW_AGGR migration drill).
                # bus overlays only + the OTA-specific config: the boot image's
                # cp_overlay must not leak in (choice lines would conflict)
                ota_fw = build_fw.build(spec.example, spec.cp_role, spec.cp_target,
                                        cp_bus_ovl + list(spec.cp_ota_ovl))
            host_inject = {spec.cp_app_to_host: ota_fw["elf"][:-4] + ".bin"}
        host_fw = build_fw.build(spec.example, spec.host_role, spec.host_target,
                                 host_ovl, inject=host_inject)
        build_fw.stash_logs(lab_tmp, cp=cp_fw, host=host_fw)
        # Prewarm pass: firmware is built + cached now; skip the emu launch so the
        # build phase never overlaps the real run's emus (cold-cache all-cores
        # builds otherwise starve the latency-sensitive emu clocks). eh.py runs a
        # prewarm pass before the emu run — see cmd_test's two-pass.
        if os.environ.get("EH_PREWARM"):
            import pytest
            pytest.skip("prewarm: firmware built, launch skipped")
        # COW level 2: each bench boots its OWN reflink copy so parallel benches
        # and NVS writes never touch the cached base.
        cp_flash = build_fw.cow_flash(cp_fw["merged"], lab_tmp)
        host_flash = build_fw.cow_flash(host_fw["merged"], lab_tmp)

        # One unique {slot, port} per bench — never shared with another pair, never
        # reused this session (see lab.alloc_bench). slot uniquifies the socket.
        res = lab.alloc_bench(worker_id)
        slot, port = res["slot"], res["port"]
        sock = lab.sock_path(worker_id, f"{spec.transport}_{slot}")  # short (sun_path cap)
        try:
            os.unlink(sock)
        except OSError:
            pass

        # Reset (host→CP) + wake (CP→host) GPIOs on both wires. On the P4-C6
        # Function-EV board every transport shares one active-low EN on GPIO 54.
        if spi:
            # SPI (GPSPI2) master↔slave. Handshake (FD only) + data-ready are the
            # slave's out-of-band GPIOs, forwarded as bridge GPIO frames; the flag
            # values differ per role (CP vs host pinout). No CP→host wake path.
            wire = "--hosted-spi-hd" if spi_hd else "--hosted-spi"
            pins = _SPI_PINS["spi_hd" if spi_hd else "spi_fd"]
            cp_sig = (["--hosted-spi-hs-gpio", pins["cp_hs"]] if pins["cp_hs"] else []) \
                   + ["--hosted-spi-dr-gpio", pins["cp_dr"]] \
                   + ["--hosted-spi-cs-gpio", pins["cp_cs"]]
            host_sig = (["--hosted-spi-hs-gpio", pins["host_hs"]] if pins["host_hs"] else []) \
                     + ["--hosted-spi-dr-gpio", pins["host_dr"]]
            cp_bridge = [wire, f"bridge:slave:{sock}", *cp_sig]
            # P4-C6 Function-EV board: one active-low EN on GPIO 54 for every transport.
            host_bridge = [wire, f"bridge:host:{sock}", *host_sig,
                           "--hosted-reset-gpio", _RESET_GPIO]
        else:
            wire = "--hosted-uart" if uart else "--hosted"
            reset_pol = []   # FUNC_BOARD reset is active-low on every transport
            # Only wire the CP→host wake GPIO when the test actually uses wake — a
            # non-wake test shouldn't require an esp-emu new enough to know the flag.
            wake_arg = ["--hosted-wake-gpio", _WAKE_GPIO] if spec.wake else []
            cp_bridge = [wire, f"bridge:slave:{sock}", *wake_arg]
            host_bridge = [wire, f"bridge:host:{sock}", "--hosted-reset-gpio", _RESET_GPIO, *reset_pol]
        # host-forward map: guest CP port → host loopback port. :22 (host-reserved)
        # backs the wake stimulus; spec.hostfwd adds CP ports for network-split
        # routing tests. Each host port sits in this bench's disjoint slot block
        # (port+1+i) so parallel benches never collide.
        fwd = {}
        if spec.wake:
            fwd[22] = port
        for i, gp in enumerate(spec.hostfwd):
            fwd[int(gp)] = port + 1 + i
        net_arg = ["--net", "user," + ",".join(
            f"hostfwd=tcp:127.0.0.1:{hp}-:{gp}" for gp, hp in fwd.items())] if fwd else []

        # BLE connect/GATT via Bumble: forward the CP's HCI to a Bumble virtual
        # controller (the host's peripheral HCI reaches it over the transport).
        _ble_port = os.environ.get("EH_BLE_HCI_PORT")
        ble_arg = ["--ble-hci", f"tcp:localhost:{_ble_port}"] if _ble_port else []
        cp = EmuDut("cp", [str(emu), "--chip", spec.cp_target, "--firmware", cp_flash,
                           "--elf", cp_fw["elf"], *cp_bridge, *ble_arg, *net_arg, "--timeout", timeout],
                    lab_tmp / f"cp_{spec.transport}.log")
        # Deterministic ordering: wait for the slave socket before the host connects
        # (polls; returns the instant it appears — no fixed sleep).
        end = time.time() + 10
        while not os.path.exists(sock) and time.time() < end and cp.proc.poll() is None:
            time.sleep(0.05)
        host = EmuDut("host", [str(emu), "--chip", spec.host_target, "--firmware", host_flash,
                               "--elf", host_fw["elf"], *host_bridge, "--timeout", timeout],
                      lab_tmp / f"host_{spec.transport}.log")

        def down():
            settle_check(host, cp)  # post-success crash catch (late panic / wake fault)
            host.stop()
            cp.stop()
            try:
                os.unlink(sock)
            except OSError:
                pass

        caps = self.caps | ({CAP_NET} if fwd else frozenset())
        return Bench(duts=[host, cp], net=Net(port, fwd) if fwd else None,
                     teardown=down, caps=caps)
