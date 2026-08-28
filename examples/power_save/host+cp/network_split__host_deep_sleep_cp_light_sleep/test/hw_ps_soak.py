#!/usr/bin/env python3
"""
Host deep-sleep / CP light-sleep soak across ALL transports, on real hardware.

Runs one transport at a time: build -> flash both -> bring up -> N cycles of
(host sleeps, CP light-sleeps, DWELL, wake packet, verify). The dwell is the
point of the exercise: it parks the pair in its low-power state long enough to
mean something to a power meter.

POWER (optional)
  A Joulescope is used if one is attached AND the `joulescope` package is
  importable; otherwise the run continues with power reporting skipped. Samples
  stream continuously to a file, and a phase log records the sleep windows, so
  the average is computed over the dwell only.

PREREQUISITES (hardware bench)
  1. Board:   CONFIG_ESP_HOSTED_P4_C5_CORE_BOARD=y on BOTH sides. That selects
              the CP host-wake GPIO (3 on this board) and every transport's pins.
  2. IDF:     v6.0.2. Each transport is built from a FRESH sdkconfig -
              `reconfigure` does NOT apply newly added defaults to an existing
              sdkconfig, so a stale one silently keeps the wrong pins.
  3. Wake:    the host wakes ONLY on a wake-up PACKET (TCP to port 22 of the host
              IP). The CP's `wake-up` CLI does NOT wake it, and the CP console is
              powered down while it light-sleeps.
  4. Network: THIS MACHINE MUST BE ON THE SAME WI-FI as the DUT. Same subnet is
              not enough - without the same L2 the SYN never arrives
              (EHOSTUNREACH) and nothing ever wakes.
  5. Creds:   choose the AP that puts the DUT on this machine's network.
  6. Logs:    the P4 console is USB-serial-JTAG; its output is buffered and deep
              sleep kills USB before it drains, so the host's lines either side
              of a sleep are LOST. Cycle accounting therefore uses the CP log and
              the opt-in counters (CONFIG_APP_PS_NVS_STATS on the host,
              CONFIG_APP_CP_PS_STATS on the CP), never host log scraping.
  7. Flashing: hold the P4 in its ROM bootloader before flashing the C5, or
              esptool reports "No serial data received".
  8. P4 download boot: holding the P4 in its bootloader latches USB-serial-JTAG
              download mode, and an RTS hard reset does NOT clear it (the port
              enumerates and esptool syncs, but the app never runs). Escape it
              with `esptool --after watchdog-reset`.
  9. esptool: in the IDF 6 env the esptool.py shebang is wrong; invoke it as
              `python -m esptool`.
 10. Joulescope: on a JS110 the instrument is the target's POWER PATH. Do not
              open/close it casually - leaving the current range open cuts power
              to the board and reboots it mid-test. `--probe` therefore never
              opens the device, and the capture opens once and holds it for the
              whole run. Closing the Joulescope UI can itself drop the target's
              supply until something re-enables the range.
"""
import argparse, errno, io, json, os, re, socket, subprocess, sys, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_book

HERE = os.path.dirname(os.path.abspath(__file__))   # <example>/test
EX   = os.path.dirname(HERE)                        # example root (has cp/, esp_host/)
REPO = os.path.abspath(os.path.join(EX, '..', '..', '..', '..'))
IDF_PY = '/Users/yogesh_1/.espressif/python_env/idf6.0_py3.14_env/bin/python'

# transport -> (cp overlay lines, host overlay lines)
TRANSPORTS = {
    'spi_fd': (['CONFIG_EH_TRANSPORT_CP_SPI=y'],
               ['CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI=y']),
    'spi_hd': (['CONFIG_EH_TRANSPORT_CP_SPI_HD=y'],
               ['CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD=y']),
    'sdio':   (['CONFIG_EH_TRANSPORT_CP_SDIO=y'],
               ['CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO=y']),
    'uart':   (['CONFIG_EH_TRANSPORT_CP_UART=y'],
               ['CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART=y']),
}
# Board profile is the committed sdkconfig.defaults.p4-c5-core-board (added to the
# SDKCONFIG_DEFAULTS chain below); these overlays carry only the test-stat opts.
COMMON_CP = ['CONFIG_APP_CP_PS_STATS=y']
COMMON_HOST = ['CONFIG_APP_PS_NVS_STATS=y']

FATAL = ['Guru Meditation', 'ESP_ERROR_CHECK failed', 'Firmware abort']


JS_APP_BIN = '/Applications/Joulescope.app/Contents/MacOS/joulescope_launcher'


def js_app_running():
    """Is the Joulescope desktop app up? It claims the device exclusively."""
    r = subprocess.run(['pgrep', '-f', 'joulescope_launcher'], capture_output=True, text=True)
    return [x for x in r.stdout.split() if x.strip()]


def power_probe(verbose=True):
    """Tri-state probe. Returns one of: 'ready', 'busy', 'absent', 'nodriver'.

    A failing probe is the NORMAL case - the Joulescope is usually detached - so
    callers must treat 'absent' as "skip power reporting", never as an error.
      absent   : no device on USB
      busy     : device present but claimed (almost always the desktop app)
      ready    : device present and claimable by this process
      nodriver : the joulescope python package is not installed
    """
    say = (lambda m: print(m, flush=True)) if verbose else (lambda m: None)
    try:
        from joulescope import scan
    except Exception as e:
        say(f'power probe: nodriver - joulescope package not importable ({e})')
        say('  install:  .deps/testvenv/bin/python -m pip install joulescope')
        return 'nodriver'
    devs = scan()
    if not devs:
        say('power probe: absent - no Joulescope on USB (normal; power reporting will be skipped)')
        return 'absent'
    name = str(devs[0])
    pids = js_app_running()
    if pids:
        say(f'power probe: busy - {name} present, Joulescope app running (pid {" ".join(pids)}); '
            'close it to free the device')
        return 'busy'
    # Deliberately NOT opening the device here.
    #
    # On a JS110 the instrument IS the DUT's power path. Opening it and closing
    # again leaves the current range open, which CUTS POWER TO THE TARGET - a
    # "harmless" probe reboots the board mid-test. Presence plus "no app holding
    # it" is enough to decide whether to try a capture; the capture itself opens
    # the device once and keeps it open for the whole run.
    say(f'power probe: ready - {name} present, nothing else holding it '
        '(not opened: opening/closing a JS110 would power-cycle the target)')
    return 'ready'


# ---------------------------------------------------------------- power (opt-in)
class Power:
    """Joulescope capture sized for MULTI-HOUR runs.

    Uses the device's on-device statistics stream (~2 Hz), not raw samples: raw
    is 2 MSa/s, i.e. ~140 GB for a five-hour soak. Each statistics record still
    carries min/mean/max computed ON THE DEVICE over its window, so short current
    bursts are not averaged away - a >300 mA Wi-Fi TX burst shows up in `max`
    even though we never store the samples it came from. The JS110's charge and
    energy accumulators give the run totals for free.

    Absent hardware, an absent package, or a device already claimed by the
    Joulescope UI all degrade to "capture off" and the soak still runs.
    """
    def __init__(self, outdir, enabled):
        self.ok = False
        self.why = 'disabled by flag'
        self.samples = os.path.join(outdir, 'power_stats.csv')
        self.phases = os.path.join(outdir, 'power_phases.csv')
        self._ph = io.open(self.phases, 'w')
        self._ph.write('t_wall,phase\n')
        self._dev = None
        if not enabled:
            return
        try:
            from joulescope import scan
        except Exception as e:
            self.why = f'joulescope package missing ({e})'
            return
        devs = scan()
        if not devs:
            self.why = 'no Joulescope attached'
            return
        self._f = io.open(self.samples, 'w')
        self._f.write('t_wall,i_mean_A,i_min_A,i_max_A,v_mean_V,charge_C,energy_J\n')
        try:
            self._dev = devs[0]
            self._dev.open()
            self._dev.parameter_set('i_range', 'auto')
            self._dev.parameter_set('v_range', '15V')
            self._dev.statistics_callback_register(self._on_stats, 'sensor')
            self._dev.start()
            self.ok = True
            self.why = f'streaming statistics from {devs[0]}'
        except Exception as e:
            # Most commonly the desktop Joulescope app already owns the device.
            hint = ' (close the Joulescope UI: only one process may claim it)' \
                   if 'claim' in str(e).lower() or '-3' in str(e) else ''
            self.why = f'open failed: {e}{hint}'
            try:
                if self._dev: self._dev.close()
            except Exception:
                pass
            self._dev = None
        return

    def _on_stats(self, st):
        try:
            sig = st.get('signals', {})
            acc = st.get('accumulators', {})
            g = lambda k, f: sig.get(k, {}).get(f, {}).get('value', float('nan'))
            self._f.write('%.3f,%.6f,%.6f,%.6f,%.4f,%.6f,%.6f\n' % (
                time.time(), g('current', 'µ'), g('current', 'min'), g('current', 'max'),
                g('voltage', 'µ'),
                acc.get('charge', {}).get('value', float('nan')),
                acc.get('energy', {}).get('value', float('nan'))))
            self._f.flush()
        except Exception:
            pass

    def mark(self, phase):
        self._ph.write(f'{time.time():.3f},{phase}\n'); self._ph.flush()

    def jls_window(self, path, seconds):
        """Record a SHORT full-rate .jls the Joulescope UI can open.

        Full rate is ~29.5 MB/s, i.e. ~532 GB for a five-hour soak - so a
        continuous recording is not an option, and reducing `sampling_frequency`
        breaks DataRecorder in this version ("stream_buffer length too small").
        Short windows at full fidelity give the actual waveform to look at
        without the terabyte: 10 s costs ~300 MB.

        The statistics stream keeps running throughout, so the run-wide averages
        and spike counts are unaffected by whether a window is being recorded.
        """
        if not self.ok:
            return None
        try:
            from joulescope.data_recorder import DataRecorder
            os.makedirs(os.path.dirname(path), exist_ok=True)
            rec = DataRecorder(path, calibration=self._dev.calibration)
            self._dev.stream_process_register(rec)
            time.sleep(seconds)
            self._dev.stream_process_unregister(rec)
            rec.close()
            return os.path.getsize(path)
        except Exception as e:
            return f'jls failed: {e}'

    def close(self):
        if self.ok:
            try: self._dev.stop()
            except Exception: pass
            # Leave the current path CONDUCTING before handing the device back:
            # on a JS110 the instrument is the target's supply, and closing with
            # the range open cuts power to the board.
            try: self._dev.parameter_set('i_range', 'auto')
            except Exception: pass
            try: self._dev.close()
            except Exception: pass
            try: self._f.close()
            except Exception: pass
        self._ph.close()

    def per_cycle(self, transport, kind='sleep'):
        """Average current for EACH cycle's phase, not pooled.

        The pooled average hides how noisy the environment is: on a busy AP a
        single 30s parked window swings by ~1 mA, which is the same size as the
        difference between transports. Reporting the spread stops a run-to-run
        ordering flip from being read as a transport property - measured: spi_hd
        4.196 mA in one run and 2.683 mA in the next, with an identical 0.37 mA
        light-sleep floor in the full-rate captures.
        """
        if not os.path.exists(self.samples):
            return []
        rows = []
        for l in io.open(self.samples).read().splitlines()[1:]:
            p = l.split(',')
            try: rows.append((float(p[0]), float(p[1])))
            except (ValueError, IndexError): continue
        ph = []
        for l in io.open(self.phases).read().splitlines()[1:]:
            t, _, name = l.partition(',')
            try: ph.append((float(t), name))
            except ValueError: pass
        out = []
        for idx, (t0, name) in enumerate(ph):
            if not name.startswith(f'{transport}/{kind}/'):
                continue
            t1 = ph[idx + 1][0] if idx + 1 < len(ph) else float('inf')
            win = [m for (t, m) in rows if t0 <= t < t1]
            if win:
                out.append(1000.0 * sum(win) / len(win))
        return out

    def summarise(self, spike_a=0.300):
        """Average / peak / spike-count per phase, joined on wall-clock time."""
        if not os.path.exists(self.samples) or os.path.getsize(self.samples) < 60:
            return None
        rows = []
        for l in io.open(self.samples).read().splitlines()[1:]:
            p = l.split(',')
            if len(p) < 4:
                continue
            try:
                rows.append((float(p[0]), float(p[1]), float(p[3])))   # t, mean, max
            except ValueError:
                continue
        ph = []
        for l in io.open(self.phases).read().splitlines()[1:]:
            t, _, name = l.partition(',')
            try: ph.append((float(t), name))
            except ValueError: pass
        out = {}
        for idx, (t0, name) in enumerate(ph):
            t1 = ph[idx + 1][0] if idx + 1 < len(ph) else float('inf')
            win = [(m, mx) for (t, m, mx) in rows if t0 <= t < t1]
            if not win:
                continue
            kind = name.split('/')[1] if '/' in name else name   # sleep | wake
            key = f"{name.split('/')[0]}/{kind}"
            d = out.setdefault(key, {'n': 0, 'sum': 0.0, 'peak': 0.0, 'spikes': 0, 'windows': 0})
            d['n'] += len(win)
            d['sum'] += sum(m for m, _ in win)
            d['peak'] = max(d['peak'], max(mx for _, mx in win))
            d['spikes'] += sum(1 for _, mx in win if mx >= spike_a)
            d['windows'] += 1
        for d in out.values():
            d['avg_mA'] = 1000.0 * d['sum'] / max(d['n'], 1)
            d['peak_mA'] = 1000.0 * d['peak']
        return out


# ------------------------------------------------------------------ build/flash
def wake_packet(ip):
    try:
        k = socket.socket(); k.settimeout(3); k.connect_ex((ip, 22)); k.close()
    except OSError:
        pass


def reachable(ip):
    """Can a wake packet from THIS machine actually arrive at the DUT?

    Nothing listens on port 22, so a reachable host answers with RST
    (ECONNREFUSED) - that is a success here. A timeout or unreachable means the
    DUT is on a subnet this machine cannot route to, which would show up later
    as "host never came back" on every single cycle and look like a transport
    bug. Check once, up front, and say so plainly.
    """
    try:
        k = socket.socket(); k.settimeout(4)
        rc = k.connect_ex((ip, 22)); k.close()
    except OSError as e:
        return False, str(e)
    if rc in (0, errno.ECONNREFUSED):
        return True, ''
    return False, os.strerror(rc) if rc else 'no answer'


def cp_boot_wake(cp_port, log=lambda m: None):
    """Reset the co-processor so its boot-wake pulse respins the host.

    The only network-independent way back: in light sleep the CP powers its
    console down, so no CLI reaches it, and a sleeping host answers nothing.
    A CP reset fires EH_CP_FEAT_HOST_PS_WAKE_HOST_ON_CP_BOOT, which pulses the
    host-wake GPIO exactly once. Safe only OUTSIDE the measured cycles: it drops
    the association and restarts the CP counters.
    """
    sh(f'{IDF_PY} -m esptool --chip esp32c5 -p {cp_port} --before default-reset '
       f'--after hard-reset read-mac', timeout=120)
    log('  CP reset: boot-wake pulse issued')


def ensure_host_awake(host_port, ip, timeout=120, cp_port=None):
    """Bring the host back before doing anything that needs its USB.

    Deep sleep powers the P4's USB-serial-JTAG down, so the port simply is not
    there - and esptool cannot flash a port that does not exist. A previous run
    (or an aborted one) routinely leaves the host asleep, so every step that
    touches the host port has to wake it first.
    """
    if os.path.exists(host_port):
        return True

    # CP first: its boot pulses the host-wake GPIO, which needs no network and no
    # correct IP, and the port is back within ~4 s. A wake packet needs the host
    # on the AP we think it is on, which is exactly what is untrue after an AP
    # change or an aborted run. Only acceptable where a CP reset is (bring-up).
    if cp_port:
        cp_boot_wake(cp_port)
        t0 = time.time()
        while time.time() - t0 < 30:
            if os.path.exists(host_port):
                return True
            time.sleep(1)

    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(host_port):
            return True
        if ip:
            wake_packet(ip)
        time.sleep(2)
    return os.path.exists(host_port)


def sh(cmd, timeout=900):
    return subprocess.run(['bash', '-lc', cmd], capture_output=True, text=True, timeout=timeout)

def write_overlay(path, lines, header):
    io.open(path, 'w').write('# ' + header + '\n' + '\n'.join(lines) + '\n')

def leave_download_mode(host_port, log=lambda m: None):
    """Force the P4 out of ROM download mode.

    Holding the P4 in its bootloader (so it stops driving the C5's lines while
    the C5 is flashed) latches USB-serial-JTAG download boot, and that latch
    survives an RTS hard reset - the port still enumerates, esptool still syncs,
    but the application never runs: no console, no transport, no association.
    Only a watchdog reset clears it. `--before default-reset` first, so this is
    deterministic whether or not the app is currently running.
    """
    sh(f'{IDF_PY} -m esptool --chip esp32p4 -p {host_port} --before default-reset '
       f'--after watchdog-reset read-mac', timeout=120)
    log('  P4 watchdog-reset out of download boot')


def in_download_mode(host_port):
    """True if the P4 is sitting in ROM download mode instead of running its app.

    A chip that answers esptool's sync WITHOUT esptool resetting it first is in
    the bootloader. This is worth distinguishing: a latched download boot looks
    exactly like a Wi-Fi/AP problem (port enumerates, no console, no traffic).
    """
    r = sh(f'{IDF_PY} -m esptool --chip esp32p4 -p {host_port} --before no-reset '
           f'--after no-reset --connect-attempts 1 --no-stub read-mac', timeout=90)
    return r.returncode == 0


def build_and_flash(t, host_port, cp_port, log, ip):
    # The host must be awake: both the "hold P4 in bootloader" step and the host
    # flash need its USB, which deep sleep powers down.
    if not ensure_host_awake(host_port, ip, cp_port=cp_port):
        log('  host USB absent and would not wake - cannot flash'); return False
    cp_ovl, host_ovl = TRANSPORTS[t]
    write_overlay(f'{EX}/cp/hw_soak.defaults', COMMON_CP + cp_ovl, f'HW soak: {t} (CP)')
    write_overlay(f'{EX}/esp_host/hw_soak.defaults', COMMON_HOST + host_ovl, f'HW soak: {t} (host)')
    # FRESH sdkconfig per transport: reconfigure will not apply new defaults.
    for d, tgt, sdk in ((f'{EX}/cp', 'esp32c5', 'sdkconfig.soak'),
                        (f'{EX}/esp_host', 'esp32p4', 'sdkconfig.soak')):
        b = 'build_soak'
        r = sh(f'. ~/esp-idf/export.sh >/dev/null 2>&1; cd {d} && rm -f {sdk} && '
               f'idf.py -B {b} -DSDKCONFIG={sdk} '
               f'-DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.{tgt};sdkconfig.defaults.p4-c5-core-board;hw_soak.defaults" '
               f'set-target {tgt} >/dev/null 2>&1 && idf.py -B {b} -DSDKCONFIG={sdk} build')
        if 'Project build complete' not in r.stdout:
            log(f'  BUILD FAILED ({d}):\n' + (r.stdout or r.stderr)[-1500:])
            return False
    # hold the P4 in its ROM bootloader so it stops driving the C5's lines
    sh(f'{IDF_PY} -m esptool --chip esp32p4 -p {host_port} --before default-reset '
       f'--after no-reset chip-id', timeout=120)
    r = sh(f'. ~/esp-idf/export.sh >/dev/null 2>&1; cd {EX}/cp && ESPBAUD=460800 '
           f'idf.py -B build_soak -DSDKCONFIG=sdkconfig.soak -p {cp_port} flash', timeout=600)
    if 'Hash of data verified' not in r.stdout:
        log('  CP FLASH FAILED:\n' + (r.stdout or r.stderr)[-1200:]); return False
    ensure_host_awake(host_port, ip, cp_port=cp_port)
    r = sh(f'. ~/esp-idf/export.sh >/dev/null 2>&1; cd {EX}/esp_host && '
           f'idf.py -B build_soak -DSDKCONFIG=sdkconfig.soak -p {host_port} flash', timeout=600)
    if 'Hash of data verified' not in r.stdout:
        log('  HOST FLASH FAILED:\n' + (r.stdout or r.stderr)[-1200:]); return False
    leave_download_mode(host_port, log)
    return True


# ------------------------------------------------------------------- serial I/O
class Serials:
    def __init__(self, cp_port, host_port):
        import serial
        self._serial = serial
        self.ports = {'cp': cp_port, 'host': host_port}
        self.buf = {'cp': [], 'host': []}
        self.h = {'cp': None, 'host': None}
        self.run = True
        for n in self.buf:
            threading.Thread(target=self._rd, args=(n,), daemon=True).start()

    def _rd(self, n):
        while self.run:
            try:
                s = self._serial.Serial(self.ports[n], 115200, timeout=0.2)
            except Exception:
                time.sleep(0.15); continue
            self.h[n] = s
            try:
                while self.run:
                    d = s.read(4096)
                    if d: self.buf[n].append(d.decode('utf-8', 'replace'))
            except Exception:
                pass
            finally:
                try: s.close()
                except Exception: pass
                self.h[n] = None; time.sleep(0.15)

    def txt(self, n): return ''.join(self.buf[n])
    def send(self, n, c):
        s = self.h.get(n)
        if not s: return False
        try: s.write((c + '\n').encode()); return True
        except Exception: return False
    def close(self): self.run = False; time.sleep(0.4)


# ------------------------------------------------------------------- the cycles
def run_transport(t, args, outdir, power, log):
    if args.skip_flash:
        # For iterating on the harness against firmware already on the chips.
        # Never use it for a certification run: nothing then proves the build
        # under test is the one that ran.
        log(f'\n===== {t}: using the firmware already flashed (--skip-flash) =====')
    else:
        log(f'\n===== {t}: build + flash =====')
        if not build_and_flash(t, args.host_port, args.cp_port, log, args.ip):
            return {'transport': t, 'ok': False, 'reason': 'build/flash'}

    S = Serials(args.cp_port, args.host_port)
    time.sleep(3)

    def dump():
        """Persist both consoles. Called on EVERY exit path: a bring-up failure
        is exactly when the logs matter most, and an early return used to throw
        them away."""
        io.open(f'{outdir}/{t}_host.log', 'w', errors='replace').write(S.txt('host'))
        io.open(f'{outdir}/{t}_cp.log', 'w', errors='replace').write(S.txt('cp'))

    # Bring-up may recover the host with a CP reset; the cycle loop must not -
    # a reset there would hide the very failure the soak is looking for.
    host_awake = lambda to: ensure_host_awake(args.host_port, args.ip, to)

    if not ensure_host_awake(args.host_port, args.ip, 90, cp_port=args.cp_port):
        dump(); S.close()
        return {'transport': t, 'ok': False, 'reason': 'host never enumerated'}
    time.sleep(4)
    S.send('host', '')
    # Only look at output produced AFTER the command: the host auto-connects to
    # its STORED credentials at boot, so the scrollback already holds an address
    # for whatever AP the last run used. Matching that gives a stale IP on the
    # wrong subnet and every wake then silently fails.
    hm, cm = len(S.txt('host')), len(S.txt('cp'))   # per-stream marks
    fresh = lambda: S.txt('host')[hm:] + S.txt('cp')[cm:]
    S.send('host', f'sta {args.ssid} {args.password}')
    t0 = time.time(); ip = None
    asked_ip = False
    while time.time() - t0 < 90:
        f = fresh()
        m = re.search(r'sta ip: (\d+\.\d+\.\d+\.\d+)', f)
        if m and m.group(1) != '0.0.0.0': ip = m.group(1); break
        # 'already on "<ssid>" - adopting' means the host kept an association it
        # already had, so no fresh DHCP line is coming: ask the netif instead.
        m = re.search(r'^ip (\d+\.\d+\.\d+\.\d+)', f, re.M)
        if m and m.group(1) != '0.0.0.0': ip = m.group(1); break
        if 'adopting' in f and not asked_ip:
            asked_ip = True
            S.send('host', 'ip')
        # A slow or missed association: re-ask periodically rather than sitting
        # out the whole 90s on one prompt that may have been eaten.
        if not asked_ip and time.time() - t0 > 30:
            asked_ip = True
            S.send('host', '')
            S.send('host', 'ip')
        time.sleep(0.4)
    if not ip:
        # Every wake in this test is a TCP packet to the DUT, so a wrong or stale
        # address turns into "host never came back" on cycle 0. Refuse to run
        # rather than blame the transport for a bring-up failure - and say WHICH
        # bring-up failure it was, since a latched download boot is impossible to
        # tell from an association failure by symptom alone.
        dump(); S.close()
        why = ('P4 in ROM download boot (app never ran)' if in_download_mode(args.host_port)
               else f'never associated to {args.ssid}')
        return {'transport': t, 'ok': False, 'reason': why}
    args.ip = ip
    ok, why = reachable(ip)
    log(f'  DUT IP = {args.ip} (reachable from here: {ok}{"" if ok else " - " + why})')
    if not ok:
        # Every wake is a TCP packet from this machine; if it cannot arrive, the
        # soak would report a transport failure for a bench routing problem.
        dump(); S.close()
        return {'transport': t, 'ok': False,
                'reason': f'DUT {ip} not reachable from this machine ({why}); '
                          f'wake packets cannot arrive - put this machine on the '
                          f'DUT\'s network'}

    fails, done = [], 0
    # The announcement is logged by DIFFERENT layers depending on the transport:
    # spi/spi_hd/uart decode it from the wire-header flags ("Host informed
    # starting to power sleep"), while sdio learns it only through the host-PS
    # feature alert ("Host Sleep"). Anchoring on the spi wording alone reports a
    # perfectly healthy sdio run as "CP never saw the PS announcement", so match
    # any layer that proves the CP was told.
    ANNOUNCE = re.compile(r'Host informed starting to power sleep'
                          r'|ehcp_host_ps: Host Sleep'
                          r'|==> Host preparing to enter power save')
    ann = lambda: len(ANNOUNCE.findall(S.txt('cp')))
    for i in range(args.cycles):
        if not host_awake(90):
            fails.append(f'cycle {i}: host never came back'); break
        time.sleep(3)
        base, before = len(S.txt('cp')), ann()
        got = False
        for _ in range(4):
            S.send('host', 'host_power_save')
            t0 = time.time()
            while time.time() - t0 < 12:
                if ann() > before: got = True; break
                time.sleep(0.2)
            if got: break
        if not got:
            fails.append(f'cycle {i}: CP never saw the PS announcement'); break
        t0 = time.time(); slept = False
        while time.time() - t0 < 45:
            seg = S.txt('cp')[base:]
            if 'Light sleep ENABLED' in seg: slept = True; break
            if any(f in seg for f in FATAL):
                fails.append(f'cycle {i}: FATAL on CP'); break
            time.sleep(0.2)
        if not slept:
            if not fails: fails.append(f'cycle {i}: CP light-sleep timeout')
            break
        t0 = time.time()
        while os.path.exists(args.host_port) and time.time() - t0 < 20: time.sleep(0.3)
        if os.path.exists(args.host_port):
            fails.append(f'cycle {i}: host did not deep-sleep'); break

        # ---- the dwell: both parked. This is the power-measurement window.
        power.mark(f'{t}/sleep/{i}')
        left = args.dwell
        if args.jls_window > 0 and args.jls_every > 0 and (i % args.jls_every) == 0:
            # a short full-rate capture inside the dwell, for eyeballing in the
            # Joulescope UI; the statistics stream keeps running regardless.
            w = min(args.jls_window, max(args.dwell - 2.0, 1.0))
            sz = power.jls_window(f'{outdir}/jls/{t}_c{i:04d}.jls', w)
            if isinstance(sz, int):
                log(f'    jls {t}_c{i:04d}.jls {sz/1e6:.0f} MB ({w:.0f}s)')
            elif sz:
                log(f'    {sz}')
            left = max(args.dwell - w, 0)
        time.sleep(left)
        power.mark(f'{t}/wake/{i}')

        woke = False
        for _ in range(12):
            wake_packet(args.ip)
            t0 = time.time()
            while time.time() - t0 < 5:
                if os.path.exists(args.host_port): woke = True; break
                time.sleep(0.25)
            if woke: break
        if not woke:
            fails.append(f'cycle {i}: host wake: USB never returned'); break
        done += 1
        if (i + 1) % 10 == 0 or i == args.cycles - 1:
            log(f'  {t}: {done}/{args.cycles} cycles')

    th, tc = S.txt('host'), S.txt('cp')
    io.open(f'{outdir}/{t}_host.log', 'w', errors='replace').write(th)
    io.open(f'{outdir}/{t}_cp.log', 'w', errors='replace').write(tc)
    S.close()
    cpc = re.findall(r'\[ps_stats\] host->\w+ sleeps=(\d+) wakes=(\d+)', tc)
    hc = re.findall(r'\[ps_stats\] boots=(\d+) wakes=(\d+) last_wake_cause=(\d+)', th)
    return {
        'transport': t, 'ok': not fails, 'cycles_done': done, 'failures': fails,
        'cp_announcements': tc.count('Host informed starting to power sleep'),
        'cp_light_sleeps': len(re.findall(r'Light sleep ENABLED', tc)),
        'cp_counters_last': cpc[-1] if cpc else None,
        'cp_counter_reset': any(int(a) < int(b) for (a, _), (b, _) in zip(cpc[1:], cpc[:-1])) if len(cpc) > 1 else False,
        'host_counters_last': hc[-1] if hc else None,
        'noresp': th.count('no response uid'), 'abort': th.count('Firmware abort'),
        'guru': th.count('Guru Meditation'),
        'notclocked': th.count('not clocked out within'),
        'notdelivered': th.count('not delivered, peer not informed'),
    }


def outdir_for(a, t):
    return os.path.join(a.out, t)


def _spread(power, t):
    """Per-cycle spread of the parked average, so the row carries its own error bar."""
    v = power.per_cycle(t, 'sleep')
    if len(v) < 2:
        return ''
    mean = sum(v) / len(v)
    sd = (sum((x - mean) ** 2 for x in v) / (len(v) - 1)) ** 0.5
    return (f'; parked per-cycle n={len(v)} mean={mean:.3f} sd={sd:.3f} '
            f'min={min(v):.3f} max={max(v):.3f} mA')


def append_book_row(a, r, power, log):
    """Append this transport's bookkeeping row NOW, not at the end of the run.

    A four-transport soak is hours long; a kill, an unplug or a crash during the
    last transport must not take the earlier transports' rows with it.
    """
    ps = power.summarise(a.spike_mA / 1000.0)
    t = r['transport']
    sleep = (ps or {}).get(f'{t}/sleep', {})
    wake = (ps or {}).get(f'{t}/wake', {})
    verdict = ''
    if sleep.get('avg_mA') is not None and a.baseline_mA > 0:
        limit = a.baseline_mA * (1.0 + a.tolerance_pct / 100.0)
        # "similar or lower than the reference" is the certification: if the
        # coprocessor really light-sleeps while the host is away, the parked
        # average cannot exceed a plain Wi-Fi power-save build by much.
        verdict = 'PASS' if sleep['avg_mA'] <= limit else 'FAIL'
        r['power_verdict'] = f"{verdict} (parked {sleep['avg_mA']:.3f} mA vs limit {limit:.3f})"
    row = dict(
        firmware='hosted', transport=t, phase='sleep+wake',
        duration_s=f'{a.cycles * (a.dwell + 15):.0f}', cycles=str(r.get('cycles_done', '')),
        dwell_s=f'{a.dwell:.0f}', ssid=a.ssid,
        cycles_ok=str(r.get('cycles_done', '')),
        cycles_failed=str(len(r.get('failures', []) or [])),
        noresp=str(r.get('noresp', '')), abort=str(r.get('abort', '')),
        guru=str(r.get('guru', '')),
        cp_counter_reset=str(r.get('cp_counter_reset', '')),
        cp_counters=str(r.get('cp_counters_last', '')),
        host_counters=str(r.get('host_counters_last', '')),
        spike_floor_mA=f'{a.spike_mA:.0f}',
        samples_csv=power.samples, artifacts_dir=outdir_for(a, t),
        notes=(f"soak: {a.cycles} cycles x {a.dwell:.0f}s dwell; "
               f"parked_avg={sleep.get('avg_mA', float('nan')):.3f} mA "
               f"parked_peak={sleep.get('peak_mA', float('nan')):.1f} mA "
               f"parked_spikes={sleep.get('spikes', '')}; "
               f"wake_avg={wake.get('avg_mA', float('nan')):.3f} mA "
               f"wake_peak={wake.get('peak_mA', float('nan')):.1f} mA; "
               f"verdict={verdict or 'n/a'}" + _spread(power, t)),
    )
    if sleep:
        row.update(avg_mA=f"{sleep['avg_mA']:.3f}", max_mA=f"{sleep['peak_mA']:.1f}",
                   spike_windows=str(sleep.get('spikes', '')))
    row.update(bench_book.row_from_config(
        f'{EX}/cp/sdkconfig.soak', f'{EX}/esp_host/sdkconfig.soak'))
    # The hosted host takes its AP from the console, not from Kconfig, so the
    # runtime SSID is the truth - put it back after the config sweep.
    row['ssid'] = a.ssid
    bench_book.append(a.book, row)
    log(f'  book row appended ({t}) -> {a.book}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--transports', default='spi_fd,spi_hd,sdio,uart')
    p.add_argument('--cycles', type=int, default=100)
    p.add_argument('--dwell', type=float, default=30.0, help='seconds parked in light sleep')
    p.add_argument('--ip', default='',
                   help='optional seed address, used only to wake the host BEFORE '
                        'the run learns the real one from the console. Leave empty: '
                        'a stale address here wakes nothing.')
    p.add_argument('--ssid', default='')
    p.add_argument('--password', default='')
    p.add_argument('--host-port', default='/dev/cu.usbmodem11101')
    p.add_argument('--cp-port', default='/dev/cu.usbserial-11201')
    p.add_argument('--skip-flash', action='store_true',
                   help='harness debugging only: run the cycles against whatever '
                        'is already on the chips')
    p.add_argument('--power', action='store_true', help='use a Joulescope if present')
    p.add_argument('--probe', action='store_true',
                   help='report Joulescope state (ready/busy/absent/nodriver) and exit')
    p.add_argument('--spike-mA', type=float, default=300.0)
    p.add_argument('--jls-window', type=float, default=0.0,
                   help='seconds of FULL-RATE .jls to record inside the dwell '
                        '(0=off). ~29.5 MB/s, so 10s ~ 300 MB - windows only, '
                        'never continuous.')
    p.add_argument('--jls-every', type=int, default=5,
                   help='record a jls window every Nth cycle')
    p.add_argument('--out', default='/tmp/hw_ps_soak')
    p.add_argument('--book', default=os.path.join(HERE, 'bench_measurements.csv'))
    p.add_argument('--baseline-mA', type=float, default=0.0,
                   help='reference average for the parked phase, e.g. the IDF '
                        'wifi/power_save figure on the same AP. The parked average '
                        'must not exceed it by more than --tolerance-pct, else CP '
                        'light sleep is not doing its job.')
    p.add_argument('--tolerance-pct', type=float, default=25.0)
    a = p.parse_args()

    if a.probe:
        state = power_probe()
        # 'absent' is not a failure: the meter is usually not connected.
        return 0 if state in ('ready', 'absent') else 2

    os.makedirs(a.out, exist_ok=True)
    logf = io.open(f'{a.out}/soak.log', 'a')
    def log(m):
        print(m, flush=True); logf.write(m + '\n'); logf.flush()

    power = Power(a.out, a.power)
    log(f'power capture: {"ON - " if power.ok else "OFF - "}{power.why}')

    results = []
    for t in [x.strip() for x in a.transports.split(',') if x.strip()]:
        if t not in TRANSPORTS:
            log(f'skip unknown transport {t}'); continue
        try:
            results.append(run_transport(t, a, a.out, power, log))
        except Exception as e:
            results.append({'transport': t, 'ok': False, 'reason': f'{type(e).__name__}: {e}'})
        log(f'  -> {json.dumps(results[-1])}')
        try:
            append_book_row(a, results[-1], power, log)
        except Exception as e:
            log(f'  book row FAILED for {t}: {type(e).__name__}: {e}')

    power.close()

    log('\n================ SUMMARY ================')
    for r in results:
        log(f'  {r["transport"]:8s} ok={r.get("ok")} cycles={r.get("cycles_done")} '
            f'noresp={r.get("noresp")} abort={r.get("abort")} guru={r.get("guru")} '
            f'cp_counters={r.get("cp_counters_last")} cp_reset={r.get("cp_counter_reset")} '
            f'host_counters={r.get("host_counters_last")} '
            f'power={r.get("power_verdict","n/a")} {r.get("failures") or r.get("reason") or ""}')
    ps = power.summarise(a.spike_mA / 1000.0)
    if ps:
        log('\n---- power by phase (avg over the phase, peak, samples >= spike floor) ----')
        for name, d in sorted(ps.items()):
            log(f'  {name:22s} avg={d["avg_mA"]:8.2f} mA  peak={d["peak_mA"]:8.2f} mA  '
                f'>= {a.spike_mA:.0f} mA: {d["spikes"]}')
    else:
        log('\npower: no samples (Joulescope absent or capture disabled)')
    io.open(f'{a.out}/results.json', 'w').write(json.dumps({'results': results, 'power': ps}, indent=1))
    log(f'\nartifacts: {a.out}')
    return 0 if all(r.get('ok') for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
