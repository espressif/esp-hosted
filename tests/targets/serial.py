"""SerialTarget — the real-hardware substrate: a host + a co-processor on two
serial ports, joined by their physical transport (SDIO / SPI / UART on the board).

    host chip  ── SDIO/SPI/UART ──  co-processor chip
       │ /dev/…usbserial (console)          │ /dev/…usbserial (console)

`make(spec)` builds both roles through the SAME COW build cache the emu uses,
flashes the pair with the existing dual-DUT flasher, then hands back a Bench of
two SerialDuts. So a HW bench and an emu bench are peers under one BenchProvider
contract — the test body can't tell which produced its `duts`.

Nothing here assumes WHICH chips are on the bench: the actual host/CP IDF targets
are declared in lab.local.json or auto-detected by probing the connected chip
(spec defaults are only the last fallback), and the board symbol makes Kconfig
auto-select that board's pins/reset/wake. Bench identity (ports, baud, chips,
board) is machine-local, resolved by `bench_config()` — never baked into the flow.
"""
import json
import os
import time

import pytest

from infra import build_fw, flasher, lab
from infra.emu_dut import settle_check
from infra.serial_dut import SerialDut

from .base import Bench, BenchProvider, BenchSpec

_DEFAULT_FLASH_BAUD = 2000000
_DEFAULT_MONITOR_BAUD = 115200


class _Bench:
    """Resolved bench identity (ports + baud + chips + flash options + board
    overlays). host_target/cp_target may be None → auto-detected from the chip."""
    __slots__ = ('host_port', 'cp_port', 'host_target', 'cp_target', 'flash_baud',
                 'monitor_baud', 'flash_cfg', 'host_board_ovl', 'cp_board_ovl')

    def __init__(self, host_port, cp_port, host_target, cp_target, flash_baud,
                 monitor_baud, flash_cfg, host_board_ovl, cp_board_ovl):
        self.host_port = host_port
        self.cp_port = cp_port
        self.host_target = host_target
        self.cp_target = cp_target
        self.flash_baud = flash_baud
        self.monitor_baud = monitor_baud
        self.flash_cfg = flash_cfg
        self.host_board_ovl = host_board_ovl
        self.cp_board_ovl = cp_board_ovl


def _read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def _ensure_idf_python():
    """Point infra.flasher's esptool calls at the IDF Python env (the one the
    build just used), by populating IDF_PYTHON_ENV_PATH once. The flasher reads
    that var to find an esptool-capable interpreter; the pytest process never
    sourced export.sh, so without this it falls back to a bare `python` that has
    no esptool. Mirrors the old runner's ensure_idf_env, scoped to what we need."""
    if os.environ.get('IDF_PYTHON_ENV_PATH'):
        return
    import subprocess
    idf = build_fw._idf_path()
    r = subprocess.run(
        ['bash', '-lc',
         f'. "{idf}/export.sh" >/dev/null 2>&1 && printf "%s" "$IDF_PYTHON_ENV_PATH"'],
        capture_output=True, text=True)
    p = r.stdout.strip()
    if p and os.path.isdir(p):
        os.environ['IDF_PYTHON_ENV_PATH'] = p


def _retry_probe(port, tries=4):
    """esptool chip/rev probe with a few retries (rides out a briefly-busy port on
    fresh enumeration). Returns the probe dict (chip/revision/error)."""
    from infra import hardware
    probe = {}
    for _ in range(tries):
        probe = hardware.eh_test_hw_probe_device(port)  # no expected_chip → no assumption
        if probe.get('chip'):
            break
        time.sleep(0.5)
    return probe


def _resolve_targets(bench, spec):
    """Resolve the actual host + CP IDF targets for THIS bench without assuming
    which chips are present: a target declared in lab.local.json short-circuits the
    probe; otherwise the connected chip is auto-detected; else the spec default.
    The host is always probed (its revision gates a P4 build), so a declared host
    target overrides the name only, not the revision read. Returns
    (host_target, cp_target, host_rev)."""
    from infra import hardware
    hp = _retry_probe(bench.host_port)
    host_target = bench.host_target or hardware.chip_to_idf_target(hp.get('chip')) or spec.host_target
    host_rev = hp.get('revision')
    if bench.cp_target:
        cp_target = bench.cp_target
    else:
        cp_target = hardware.chip_to_idf_target(_retry_probe(bench.cp_port).get('chip')) or spec.cp_target
    return host_target, cp_target, host_rev


def _rev_overlay(host_target, host_rev, port):
    """For an early ESP32-P4 (rev <3.0), unlock <3.0 support. IDF v5.5.4 defaults P4
    builds to REV_MIN v3.1; on a v0.x/v1.x board the bootloader then can't execute
    (illegal instruction at entry). Only applies to P4 — no assumption the host is
    one. Fails loud if a P4's rev can't be read (build-critical)."""
    if host_target != 'esp32p4':
        return []
    if not host_rev:
        raise RuntimeError(
            f"could not read ESP32-P4 revision on {port}; rev is build-critical "
            f"(a <3.0 P4 needs CONFIG_ESP32P4_SELECTS_REV_LESS_V3)")
    try:
        return ['CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y'] if float(host_rev) < 3.0 else []
    except ValueError:
        return []


def bench_config():
    """Resolve the physical bench, machine-local, in precedence order:
      1. env vars   EH_HOST_PORT / EH_CP_PORT / EH_FLASH_BAUD / EH_MONITOR_BAUD
      2. tests/lab.local.json  (gitignored, per-machine)   {host_port, cp_port, …}
      3. tests/env.json  ["hardware"] block  (shared default)
    Skips the test cleanly if no bench is configured, so `eh.py test hw` on a
    machine with no boards is a skip, not an error."""
    env = os.environ
    local = _read_json(lab._REPO / "tests" / "lab.local.json")
    shared = _read_json(lab._REPO / "tests" / "env.json").get("hardware", {})

    host_port = env.get("EH_HOST_PORT") or local.get("host_port") or shared.get("host_port")
    cp_port = (env.get("EH_CP_PORT") or env.get("EH_SLAVE_PORT")
               or local.get("cp_port") or shared.get("slave_port"))
    if not host_port or not cp_port:
        pytest.skip("HW bench not configured — set EH_HOST_PORT/EH_CP_PORT, "
                    "tests/lab.local.json, or tests/env.json[hardware]")

    # Chips are a bench property: declared here to skip the probe, else auto-detected
    # from the connected silicon (None → SerialTarget probes). Never assumed.
    host_target = env.get("EH_HOST_TARGET") or local.get("host_target")
    cp_target = env.get("EH_CP_TARGET") or local.get("cp_target")

    flash_baud = int(env.get("EH_FLASH_BAUD") or local.get("flash_baud")
                     or shared.get("flash_baud") or _DEFAULT_FLASH_BAUD)
    monitor_baud = int(env.get("EH_MONITOR_BAUD") or local.get("monitor_baud")
                       or _DEFAULT_MONITOR_BAUD)
    # Flash strategy defaults let the flasher's hash cache skip unchanged binaries
    # (build_fw's COW dirs are stable per config, so skip-detection works across
    # runs). Override via env.json["flash"].
    envj = _read_json(lab._REPO / "tests" / "env.json")
    flash_json = envj.get("flash", {})
    # Fast by default: hash-skip lets unchanged binaries just reset instead of
    # reflash, so a suite sharing one firmware pays the flash cost once. Force a
    # full flash only on demand (EH_FORCE_FLASH / lab.local.json force_full_flash).
    force_full = bool(env.get("EH_FORCE_FLASH") or local.get("force_full_flash", False))
    flash_cfg = {
        'strategy': flash_json.get('strategy', 'sequential'),
        'baud': flash_baud,
        'force_full_flash': force_full,
        'use_diff': flash_json.get('use_diff', False),
    }
    # Board config: real boards need their pin/PSRAM/bootloader Kconfig or they
    # won't boot. env.json["build"] carries the per-board overlay the old runner
    # applied; shared sdkconfig_optimizations go to both roles.
    build_json = envj.get("build", {})
    opt = build_json.get("sdkconfig_optimizations", []) or []
    # Board type is a per-bench fact: one board symbol (e.g. ESP_HOSTED_P4_C6_CORE_BOARD)
    # makes Kconfig auto-select that board's SDIO slot/pins, reset line + polarity, and
    # host-wake pin. It differs per physical bench, so lab.local.json OVERRIDES the shared
    # env.json default. host_extra/cp_extra layer any further per-bench Kconfig on top.
    host_extra = local.get("host_extra", []) or []
    cp_extra = local.get("cp_extra", []) or []
    host_board = local.get("host_board_sdkconfig") or build_json.get("host_board_sdkconfig", []) or []
    cp_board = local.get("cp_board_sdkconfig") or build_json.get("slave_board_sdkconfig", []) or []
    host_board_ovl = host_board + host_extra + opt
    cp_board_ovl = cp_board + cp_extra + opt
    return _Bench(host_port, cp_port, host_target, cp_target, flash_baud,
                  monitor_baud, flash_cfg, host_board_ovl, cp_board_ovl)


def _transport_overlays(transport):
    """(cp_overlay, host_overlay) sdkconfig lines selecting the runtime bus for a
    real-HW build. sdio is the neutral default (no overlay). SPI/SPI-HD only
    select the bus — the board symbol (in *_board_ovl) supplies the pins/reset
    (host/Kconfig.ext defines them per board + CP target). spi_hd_{1,2,4}
    additionally pin the data-line count on both ends. Kept in lockstep with
    tests/targets/emu.py so hw and emu build identical configs."""
    if transport == "uart":
        return (["CONFIG_EH_TRANSPORT_CP_UART=y"],
                ["CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART=y"])
    if transport == "spi_fd":
        return (["CONFIG_EH_TRANSPORT_CP_SPI=y"],
                ["CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI=y"])
    if transport.startswith("spi_hd"):
        cp = ["CONFIG_EH_TRANSPORT_CP_SPI_HD=y"]
        host = ["CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD=y"]
        lines = transport.rsplit("_", 1)[1] if transport != "spi_hd" else None
        if lines in ("1", "2", "4"):
            suffix = "1_DATA_LINE" if lines == "1" else f"{lines}_DATA_LINES"
            cp.append(f"CONFIG_EH_TRANSPORT_CP_SPI_HD_{suffix}=y")
            host.append(f"CONFIG_ESP_HOSTED_HOST_SPI_HD_{suffix}=y")
        return (cp, host)
    return ([], [])  # sdio


class SerialTarget(BenchProvider):
    def __init__(self):
        # Detect this physical bench's properties at runtime from its machine-local
        # config — the substrate reports what it is, tests don't assume.
        local = _read_json(lab._REPO / "tests" / "lab.local.json")
        tr = os.environ.get("EH_TRANSPORTS") or local.get("transports")
        if isinstance(tr, str):
            self.transports = frozenset(t.strip() for t in tr.split(",") if t.strip())
        elif tr:
            self.transports = frozenset(tr)
        else:
            self.transports = frozenset({"sdio"})  # a bench is at least SDIO-wired
        # Feature caps the board physically has (e.g. "wake" GPIO, "coex_wired").
        # Default none — the P4-C6 Core board has neither; declare in lab.local.json
        # for a board that does.
        self.caps = frozenset(local.get("caps", []) or [])

    def make(self, spec: BenchSpec, *, worker_id, lab_tmp) -> Bench:
        bench = bench_config()
        _ensure_idf_python()  # esptool on PATH for the chip/rev probe + the flasher
        # Resolve the actual chips from the bench (declared or probed), not spec
        # defaults — the substrate adapts to whatever silicon is connected.
        host_target, cp_target, host_rev = _resolve_targets(bench, spec)
        rev_ovl = _rev_overlay(host_target, host_rev, bench.host_port)
        # Per-transport bus overlay (sdio = neutral default → none). SPI/SPI-HD
        # select the bus; the board symbol in *_board_ovl auto-selects that board's
        # SPI pins + reset line (host/Kconfig.ext, per board + CP target).
        cp_x, host_x = _transport_overlays(spec.transport)
        cp_ovl = cp_x + bench.cp_board_ovl
        # Ordered layers: transport + board + rev + bench(slave chip) + case.
        # slave_overlay tells the host which CP it drives (its SoC caps), derived
        # from this bench's CP — not pinned in the example.
        host_ovl = (host_x + bench.host_board_ovl + rev_ovl
                    + build_fw.slave_overlay(cp_target) + list(spec.extra_ovl))

        cp_fw = build_fw.build(spec.example, spec.cp_role, cp_target, cp_ovl)
        host_fw = build_fw.build(spec.example, spec.host_role, host_target, host_ovl)
        build_fw.stash_logs(lab_tmp, cp=cp_fw, host=host_fw)
        cp_build_dir = os.path.join(cp_fw["dir"], "build")
        host_build_dir = os.path.join(host_fw["dir"], "build")

        # Board-swap safety: the flasher's hash cache is keyed on PORT, so a
        # different chip on the same port (a board swap) would get an app-only
        # flash and be left "not bootable" (stale/absent bootloader). If the chip
        # signature changed since the last flash on this bench, force a FULL flash.
        # Cheap — uses the rev already probed, no extra I/O.
        sig = f"{host_target}|{host_rev}|{cp_target}|{'+'.join(bench.host_board_ovl)}"
        sig_file = lab.WORK / ".bench_sig"
        try:
            swapped = sig_file.read_text().strip() != sig
        except OSError:
            swapped = True
        flash_cfg = dict(bench.flash_cfg)
        if swapped:
            flash_cfg['force_full_flash'] = True

        os.environ['FLASH_BAUD'] = str(bench.flash_baud)
        ok, flog = flasher.eh_test_flash_pair(
            bench.host_port, bench.cp_port, host_target, cp_target,
            cp_build_dir, host_build_dir, flash_cfg=flash_cfg)
        (lab_tmp / "flash.log").write_text("\n".join(flog))
        if not ok:
            raise RuntimeError(f"flash failed: {flog[-1] if flog else '?'} "
                               f"(see {lab_tmp / 'flash.log'})")
        try:  # record the signature only after a good flash
            sig_file.parent.mkdir(parents=True, exist_ok=True)
            sig_file.write_text(sig)
        except OSError:
            pass

        # Open the console monitors, then reset CP-first so the CP is up when the
        # host boots and probes the transport (mirrors the emu launch ordering).
        cp = self._open(spec, "cp", bench.cp_port, bench.monitor_baud, lab_tmp / "cp.log")
        host = self._open(spec, "host", bench.host_port, bench.monitor_baud, lab_tmp / "host.log")
        cp.reset()
        time.sleep(0.1)
        host.reset()

        def down():
            # Always release the ports, even if settle_check raises on a late crash —
            # otherwise a failed teardown leaks the serial handles and the next run's
            # probe/flash hits a busy port.
            try:
                settle_check(host, cp)
            finally:
                host.stop()
                cp.stop()

        return Bench(duts=[host, cp], net=None, teardown=down, caps=self.caps)

    @staticmethod
    def _open(spec, name, port, baud, logpath, retries=5):
        """Open the console just after esptool released the port; retry briefly to
        ride out the USB-serial re-enumeration window on macOS/Linux."""
        import serial
        last = None
        for _ in range(retries):
            try:
                return SerialDut(name, port, baud, logpath)
            except (serial.SerialException, OSError) as e:
                last = e
                time.sleep(0.5)
        raise RuntimeError(f"could not open {name} console at {port}: {last}")
