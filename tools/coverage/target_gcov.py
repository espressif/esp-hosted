#!/usr/bin/env python3
"""Dump on-target gcov (.gcda) from an ESP CP/MCU firmware via OpenOCD over the
chip's built-in USB-Serial-JTAG — no external JTAG probe.

Prereq: the firmware was built coverage-instrumented (build_fw.gcov_overlay():
APPTRACE_GCOV + whole-project --coverage) and is flashed + running.

    python3 tools/coverage/target_gcov.py --target esp32c6 --build-dir <scratch>/build

Then union the produced .gcda with the Linux-host .gcda:
    python3 tools/coverage/aggregate_gcov.py <repo> <scratch>/build [linux-build-cov ...]

HARDWARE NOTE: JTAG rides the chip's NATIVE USB-C port (shows up as a
/dev/cu.usbmodem* on macOS, /dev/ttyACM* on Linux) — NOT the UART bridge used
for flashing (/dev/cu.usbserial-*). Connect that native port to the machine that
BUILT the firmware: `esp gcov` writes .gcda to the absolute build-dir paths baked
into the ELF, so OpenOCD must run where those paths exist.
"""
import argparse
import os
import subprocess
import sys


def dump(target, build_dir, *, serial=None, openocd_cfg=None, timeout=60):
    """One-shot OpenOCD: attach over USB-JTAG, `esp gcov` (instant, no-halt dump),
    exit. Returns (ok, output). .gcda lands at the ELF's baked build paths."""
    cfg = openocd_cfg or f"board/{target}-builtin.cfg"
    cmds = ["init"]
    # `esp gcov` (no 'dump') is the instant run-time variant: it needs no app
    # cooperation and doesn't halt the CPU — safe to call mid-test.
    cmds += ["esp gcov", "exit"]
    argv = ["openocd", "-f", cfg]
    if serial:  # disambiguate when several USB-JTAG devices are attached
        argv += ["-c", f"adapter serial {serial}"]
    for c in cmds:
        argv += ["-c", c]
    env = dict(os.environ)
    # gcc records absolute .gcda paths; run from build_dir so any relative ones
    # (and the CMakeFiles/*.dir tree) resolve too.
    try:
        r = subprocess.run(argv, cwd=build_dir, env=env, text=True,
                           capture_output=True, timeout=timeout)
    except FileNotFoundError:
        return False, ("openocd not found — source the IDF export script first "
                       "(. $IDF_PATH/export.sh)")
    except subprocess.TimeoutExpired:
        return False, f"openocd timed out after {timeout}s (JTAG not connected?)"
    out = (r.stdout or "") + (r.stderr or "")
    ok = r.returncode == 0 and ".gcda" in out
    return ok, out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True, help="esp32c5 | esp32c6 | esp32p4 | …")
    ap.add_argument("--build-dir", required=True, help="the firmware's build/ dir")
    ap.add_argument("--serial", help="USB-JTAG adapter serial (multi-device benches)")
    ap.add_argument("--openocd-cfg", help="override the board cfg (default board/<target>-builtin.cfg)")
    ap.add_argument("--timeout", type=int, default=60)
    a = ap.parse_args()
    ok, out = dump(a.target, a.build_dir, serial=a.serial,
                   openocd_cfg=a.openocd_cfg, timeout=a.timeout)
    sys.stderr.write(out + "\n")
    if ok:
        print("gcov dump OK — .gcda written under the ELF's build paths")
        return 0
    print("gcov dump FAILED — see openocd output above", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
