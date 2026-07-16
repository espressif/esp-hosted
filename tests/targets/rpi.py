"""RpiTarget — the distributed real-HW Linux substrate (EH_SUBSTRATE=rpi).

A bench that spans TWO machines (see benches.md `rpi-sdio-c5`):
  - control machine (this host): builds + flashes the C5 CP over its USB-UART,
    and drives everything over SSH.
  - RPi (the Linux host DUT): builds the userspace c_app, loads the kmod, runs
    the host app over SDIO to the C5.

Flow (automates the proven manual bring-up):
  1. build the C5 CP firmware here (CP_FOR_LINUX_802_3 + V2 + SDIO)
  2. unload the kmod on the RPi (frees the C5's SDIO bus for flashing)
  3. flash the C5 CP from here over the C5's flash-UART
  4. build the c_app ON the RPi over SSH, with the slave chip injected as an
     overlay via build_fw.slave_overlay (bench-derived, not an example pin)
  5. load the kmod on the RPi (resets + handshakes the C5)
  6. run the c_app on the RPi → an EmuDut backed by the SSH process; the C5
     console (on this host's UART) is a SerialDut for crash visibility

Config: `tests/lab.local.json` → "rpi" block (see _rpi_config). Nothing is
assumed; a missing block skips the substrate.
"""
import json
import os
import subprocess
import time

import pytest

from infra import build_fw, flasher, lab
from infra.emu_dut import EmuDut
from infra.serial_dut import SerialDut

from .base import CAP_NET, Bench, BenchProvider, BenchSpec


def _rpi_config():
    """RPi bench properties from tests/lab.local.json["rpi"]. Required: ssh, repo,
    cp_flash_port. Optional: cp_target (default esp32c5), cp_console_port,
    kmod_scripts, kmod_bus (default sdio), wifi_ap{ssid,password}, and for the
    SPI wire spi{host{spi_bus,spi_cs,spi_mode,spi_handshake,spi_dataready,
    clockspeed}, cp_gpio{handshake,data_ready}} — consumed only when a test's
    transport is spi (see make())."""
    try:
        local = json.loads((lab._REPO / "tests" / "lab.local.json").read_text())
    except (OSError, ValueError):
        local = {}
    rpi = local.get("rpi") or {}
    if not (rpi.get("ssh") and rpi.get("repo") and rpi.get("cp_flash_port")):
        pytest.skip("RPi bench not configured — add an 'rpi' block "
                    "{ssh, repo, cp_flash_port, ...} to tests/lab.local.json")
    rpi.setdefault("cp_target", "esp32c5")
    rpi.setdefault("kmod_scripts", "host/linux/eh_host_linux_kmod/scripts")
    rpi.setdefault("kmod_bus", "sdio")
    return rpi


def _ssh(host, remote_cmd, logf=None, check=True):
    """Run a command on the RPi over SSH (non-interactive, key-based)."""
    argv = ["ssh", "-o", "BatchMode=yes", host, remote_cmd]
    r = subprocess.run(argv, stdout=(logf or subprocess.PIPE),
                       stderr=subprocess.STDOUT, text=True)
    if check and r.returncode != 0:
        raise RuntimeError(f"rpi ssh failed ({r.returncode}): {remote_cmd}\n"
                           f"{(r.stdout or '')[-2000:]}")
    return r


def _scp(local, remote):
    """Copy a file to the RPi (robust for content the shell would mangle)."""
    r = subprocess.run(["scp", "-o", "BatchMode=yes", str(local), remote],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"scp failed: {local} -> {remote}\n{r.stderr}")


class RpiTarget(BenchProvider):
    # Real C5↔RPi wire: SDIO (dtoverlay=sdio) or SPI (dtparam=spi=on) — the kmod
    # drives either; the test's spec.transport picks it (bench must be wired for it).
    transports = frozenset({"sdio", "spi_fd"})
    caps = frozenset({CAP_NET})        # real Linux host on a routable net; no wake GPIO

    def make(self, spec: BenchSpec, *, worker_id, lab_tmp) -> Bench:
        rpi = _rpi_config()
        ssh, repo = rpi["ssh"], rpi["repo"]
        cp_target = rpi["cp_target"]
        bus = spec.transport if spec.transport in self.transports else rpi["kmod_bus"]
        # The transport id is "spi_fd" (clear vs spi_hd); the kmod / lab wiring
        # block / CP Kconfig all speak the bus name "spi".
        if bus == "spi_fd":
            bus = "spi"
        capp_dir = f"{repo}/examples/{spec.example}/linux_802_3_host/c_app"

        # 1) C5 CP firmware (control side): Linux 802.3 (→ V2 by default) on the
        # requested wire (SDIO or SPI) — matches the kmod --bus below.
        cp_xport = ("CONFIG_ESP_HOSTED_TRANSPORT_CP_SPI=y" if bus == "spi"
                    else "CONFIG_ESP_HOSTED_TRANSPORT_CP_SDIO=y")
        cp_ovl = ["# CONFIG_ESP_HOSTED_CP_FOR_MCU is not set",
                  "CONFIG_ESP_HOSTED_CP_FOR_LINUX_802_3=y", cp_xport]
        # SPI: the C5-side handshake/data-ready GPIOs must match the bench wiring
        # (they differ from the SDIO schematic). Inject them from the bench config
        # so the CP end matches the kmod (host) end set in step 5 — bench-driven,
        # not an example pin.
        spi_cfg = (rpi.get("spi") or {}) if bus == "spi" else {}
        cp_gpio = spi_cfg.get("cp_gpio") or {}
        _cp_spi_sym = {"mosi": "MOSI", "miso": "MISO", "clk": "CLK", "cs": "CS",
                       "handshake": "HANDSHAKE", "data_ready": "DATA_READY"}
        for key, sym in _cp_spi_sym.items():
            if key in cp_gpio:
                cp_ovl.append(f"CONFIG_EH_TRANSPORT_CP_SPI_GPIO_{sym}={cp_gpio[key]}")
        cp_fw = build_fw.build(spec.example, "cp", cp_target, cp_ovl)
        build_fw.stash_logs(lab_tmp, cp=cp_fw)

        # 2) free the C5 for flashing (kmod drives its reset/SDIO otherwise)
        scripts = f"{repo}/{rpi['kmod_scripts']}"
        # Kill any orphaned c_app from a prior run FIRST: EmuDut.stop() kills the
        # local ssh, but the remote sudo'd c_app survives (SSH doesn't forward the
        # signal) and holds /dev/esps0 — then rmmod fails and the next load.sh
        # dies "File exists". Match by the example's build path, with a
        # self-excluding regex ([b]uild) so pkill doesn't match its OWN ssh
        # command line (which contains the pattern) and kill the session (ssh 255).
        _pkill = f"sudo pkill -f '{capp_dir}/[b]uild' 2>/dev/null; sleep 1; "
        _ssh(ssh, _pkill + "sudo rmmod esp32_sdio 2>/dev/null; true")

        # 3) flash the C5 CP from here over its flash-UART
        ok, msg, _ = flasher._flash_device(rpi["cp_flash_port"], cp_target,
                                           os.path.join(cp_fw["dir"], "build"),
                                           after="hard_reset")
        if not ok:
            raise RuntimeError(f"C5 CP flash failed: {msg}")
        # Let the freshly-flashed CP complete its FIRST boot (PHY calibration is
        # saved to NVS on first boot). If the kmod resets it mid-calibration, the
        # first esp_wifi_init RPC hits a not-ready wifi stack. After this, the
        # kmod's reset boots a warm, calibrated chip.
        time.sleep(12)

        # 4) build the c_app ON the RPi with the slave chip as an overlay layer
        #    (bench-derived) + the case layer (wifi creds etc.). Out-of-tree isn't
        #    available for the remote in-place build, so drop any stale sdkconfig
        #    first so the overlay/defaults regenerate.
        ap = rpi.get("wifi_ap") or {}
        case_ovl = []
        if ap.get("ssid"):
            case_ovl += [f'CONFIG_EH_EXAMPLE_WIFI_SSID="{ap["ssid"]}"',
                         f'CONFIG_EH_EXAMPLE_WIFI_PASSWORD="{ap.get("password", "")}"',
                         "CONFIG_EH_EXAMPLE_WIFI_MAXIMUM_RETRY=100"]
        overlay = build_fw.slave_overlay(cp_target) + list(spec.extra_ovl) + case_ovl
        ovl_remote = "/tmp/eh_rpi_ovl.defaults"
        # scp the overlay (its lines contain '"' from the creds — don't shell-quote)
        ovl_local = lab_tmp / "rpi_ovl.defaults"
        ovl_local.write_text("\n".join(overlay) + "\n")
        _scp(ovl_local, f"{ssh}:{ovl_remote}")
        with open(lab_tmp / "host_build.log", "w") as blog:
            _ssh(ssh,
                 f'cd {capp_dir} && rm -f sdkconfig && '
                 f'bash -lc "source {repo}/export.sh >/dev/null 2>&1; '
                 f'EH_SDKCONFIG_OVERLAY={ovl_remote} make build"',
                 logf=blog)

        # 5) load the kmod (resets + handshakes the C5), then WAIT for the char
        # device — load.sh returns after insmod, but /dev/esps0 only appears once
        # the slave-up handshake completes (~1-3s). Racing it => the c_app opens a
        # missing /dev/esps0 and aborts.
        # SPI: hand the kmod the RPi-side GPIOs + bus/cs/mode (load.sh --spi-*).
        # These are the host end of the same wires the CP overlay set above.
        load_args = f"--bus {bus}"
        if bus == "spi":
            h = spi_cfg.get("host") or {}
            _spi_flag = {"spi_bus": "--spi-bus", "spi_cs": "--spi-cs",
                         "spi_mode": "--spi-mode", "spi_handshake": "--spi-handshake",
                         "spi_dataready": "--spi-dataready", "clockspeed": "--clock-mhz"}
            for key, flag in _spi_flag.items():
                if key in h:
                    load_args += f" {flag} {h[key]}"
        _ssh(ssh, f"cd {scripts} && sudo ./load.sh {load_args}")
        _ssh(ssh, "for i in $(seq 1 30); do [ -e /dev/esps0 ] && break; sleep 0.5; "
                  "done; [ -e /dev/esps0 ] || { echo 'no /dev/esps0'; exit 1; }; "
                  "sleep 8")  # settle: slave-up (=> /dev/esps0) precedes the CP's
                  # first-boot PHY calibration after a fresh flash; give the wifi
                  # stack time to be RPC-ready before esp_wifi_init.

        # 6) DUTs: host = the c_app over SSH; cp = C5 console (if wired here).
        # `make build` doesn't stage to bin/ (that's eh.py run's job) — the binary
        # is under build/; discover it on the RPi.
        r = _ssh(ssh, f"find {capp_dir}/build -type f -perm -111 "
                      f"\\( -name '*c_app' -o -name '*_app' \\) | head -1")
        capp_bin = (r.stdout or "").strip()
        if not capp_bin:
            raise RuntimeError(f"no c_app binary under {capp_dir}/build on the RPi")
        host = EmuDut("host",
                      ["ssh", "-o", "BatchMode=yes", ssh,
                       f"cd {capp_dir} && sudo stdbuf -oL {capp_bin}"],
                      lab_tmp / "host.log")
        cp = None
        # A cp console is only safe on a SEPARATE UART: opening the C5's FLASH
        # UART pulses DTR/RTS (wired to EN/IO0), which RESETS the CP mid-run —
        # so never attach a console on the flash port.
        con = rpi.get("cp_console_port")
        if con and con != rpi["cp_flash_port"] and os.path.exists(con):
            cp = SerialDut("cp", con, 115200, lab_tmp / "cp.log")

        def down():
            host.stop()  # kills the LOCAL ssh; the remote c_app is orphaned...
            if cp:
                cp.stop()
            # ...so kill it on the RPi, then unload — leaving neither behind.
            _ssh(ssh, _pkill + "sudo rmmod esp32_sdio 2>/dev/null; true", check=False)

        duts = [host] + ([cp] if cp else [])
        return Bench(duts=duts, net=None, teardown=down, caps=self.caps)
