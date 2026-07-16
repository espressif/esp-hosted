"""LinuxTarget — the userspace Linux-host substrate: the real vserial c_app (no
kernel module, no mock) talking to an emulated C6 CP through the vserial↔eh_frame
shim over a PTY:

    linux c_app (EH_ESPS_DEV=<pty>)  <->  shim  <->  esp-emu C6 (--hosted-uart)

`duts` is [host, cp]; the linux `dut` fixture exposes just the host. Absorbs the
emu-linux conftest's `linux_bench` launch logic.
"""
import os
import subprocess
import sys
import time

from infra import build_fw, lab
from infra.emu_dut import EmuDut, emu_bin, settle_check

from .base import Bench, BenchProvider, BenchSpec

_SHIM = lab._REPO / "tools" / "eh_testlab" / "vserial_shim.py"


class LinuxTarget(BenchProvider):
    def make(self, spec: BenchSpec, *, worker_id, lab_tmp) -> Bench:
        # The posix Linux host can't build on macOS: port/idf_components uses
        # pthread_mutex_timedlock / pthread_condattr_setclock, which are Linux-glibc
        # POSIX APIs absent from Darwin libc. Skip defensively here (runs on Linux CI).
        if sys.platform == "darwin":
            import pytest
            pytest.skip("emu-linux posix host cannot build on macOS "
                        "(pthread_mutex_timedlock / pthread_condattr_setclock are "
                        "Linux-only) — runs on Linux CI")
        emu = emu_bin()
        cp_target = "esp32c6"  # the emulated CP chip (bench property)
        cp_fw = build_fw.build(spec.example, "cp", cp_target, ["CONFIG_EH_TRANSPORT_CP_UART=y"])
        # Ordered config layers, later wins: bench (the CP chip → slave SoC caps)
        # then case (spec.extra_ovl, e.g. wifi creds). No per-example pins.
        host_ovl = build_fw.slave_overlay(cp_target) + list(spec.extra_ovl)
        capp = build_fw.build_linux_host(spec.example, "linux_802_3_host/c_app",
                                         overlay=host_ovl)
        build_fw.stash_logs(lab_tmp, cp=cp_fw, host=capp)
        # Prewarm pass: firmware built + cached; skip launch so builds never
        # overlap the emu run (see eh.py cmd_test two-pass).
        if os.environ.get("EH_PREWARM"):
            import pytest
            pytest.skip("prewarm: firmware built, launch skipped")
        cp_flash = build_fw.cow_flash(cp_fw["merged"], lab_tmp)

        sock = lab.sock_path(worker_id, f"lin_{lab.alloc_bench(worker_id)['slot']}")  # unique per bench
        pty = str(lab_tmp / "esps_pty")
        for p in (sock, pty):
            try:
                os.unlink(p)
            except OSError:
                pass

        # 1) CP over the UART bridge (binds the socket)
        cp = EmuDut("cp", [str(emu), "--chip", "esp32c6", "--firmware", cp_flash,
                           "--elf", cp_fw["elf"], "--hosted-uart", f"bridge:slave:{sock}",
                           "--timeout", os.environ.get("EH_EMU_TIMEOUT", "60s")],
                    lab_tmp / "cp.log")
        end = time.time() + 10
        while not os.path.exists(sock) and time.time() < end and cp.proc.poll() is None:
            time.sleep(0.05)

        # 2) shim: connects to the CP socket, presents a PTY the vserial host opens
        shim = subprocess.Popen([sys.executable, str(_SHIM), sock, pty],
                                stdout=open(lab_tmp / "shim.log", "w"),
                                stderr=subprocess.STDOUT)
        end = time.time() + 10
        while not os.path.exists(pty) and time.time() < end and shim.poll() is None:
            time.sleep(0.05)

        # 3) the real Linux host (line-buffered so its printf is visible live)
        env = dict(os.environ, EH_ESPS_DEV=pty)
        host = EmuDut("host", ["stdbuf", "-oL", capp["bin"]], lab_tmp / "host.log", env=env)

        def down():
            settle_check(cp, host)  # post-success crash catch
            cp.stop()
            host.stop()
            shim.terminate()
            try:
                os.unlink(sock)
            except OSError:
                pass

        return Bench(duts=[host, cp], net=None, teardown=down)
