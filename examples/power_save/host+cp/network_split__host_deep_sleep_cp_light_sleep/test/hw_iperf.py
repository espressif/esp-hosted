#!/usr/bin/env python3
"""iperf throughput matrix against a DUT console, for this example's boards.

Works for any firmware that registers the espressif/iperf-cmd console (the
hosted host here, or a native-Wi-Fi chip running IDF's wifi/iperf example) —
the console syntax is identical. TCP + UDP, DUT-TX + DUT-RX, per AP.

The measuring machine is assumed dual-homed (e.g. Wi-Fi on one subnet, Ethernet
on the AP's subnet); pass --bind so its iperf uses the interface that actually
reaches the DUT and traffic can't slip onto the wrong path. Both ends are
captured; the RECEIVER side is reported (authoritative, esp. for lossy UDP).

Note the -b asymmetry: the espressif iperf-cmd wants a bare integer (Mbits),
IDF-side; the host iperf (iperf 2.x) wants the "100M" suffix. Handled below.

  hw_iperf.py --port /dev/cu.usbmodemXXX --label hosted --transport sdio \\
              --bind 192.168.50.22 --aps ESP_TEST_2G:pass,ESP_TEST_5G:pass
"""
import argparse, os, re, socket, subprocess, sys, threading, time, errno

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bench_book


class DutConsole:
    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.buf = []
        self._go = True
        threading.Thread(target=self._rd, daemon=True).start()

    def _rd(self):
        while self._go:
            try:
                ln = self.ser.readline().decode('utf-8', 'replace')
                if ln:
                    self.buf.append(ln.rstrip())
            except Exception:
                break

    def send(self, cmd):
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.3)

    def mark(self):
        return len(self.buf)

    def since(self, m):
        return self.buf[m:]

    def close(self):
        self._go = False
        time.sleep(0.4)
        self.ser.close()


def summary_mbps(lines):
    """Cumulative-summary Mbits/sec ('0.0-<t> sec' line) if present, else last."""
    cum = last = None
    for ln in lines:
        for m in re.finditer(r'([\d.]+)\s*Mbits/sec', ln):
            last = float(m.group(1))
            if re.search(r'0\.0+-\s*[\d.]+\s*sec', ln):
                cum = float(m.group(1))
    return cum if cum is not None else last


def reachable(ip, bind, port=5001, timeout=2.0):
    """True if a TCP connect to ip:port from `bind` connects or is refused
    (host is there); False on timeout / no route (stale or unreachable IP)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((bind, 0))
        s.settimeout(timeout)
        rc = s.connect_ex((ip, port))
        return rc == 0 or rc == errno.ECONNREFUSED
    except OSError:
        return False
    finally:
        s.close()


def _ip_in(ln):
    """Extract a fresh got-ip from a console line, across firmwares:
    hosted esp_netif_handlers 'sta ip: X'; hosted app_wifi 'IP_EVENT_STA_GOT_IP X';
    IDF wifi-cmd 'IP_EVENT_STA_GOT_IP: ... address: X' / '- IPv4 address: X'."""
    if re.search(r'GOT_IP|got ip|sta ip:|IPv4 address', ln):
        m = re.search(r'(\d+\.\d+\.\d+\.\d+)', ln)
        if m and m.group(1) != '0.0.0.0':
            return m.group(1)
    return None


def associate(dut, ssid, pw, bind, timeout=40):
    """Send `sta`, return (ip, reason). reason is '' on success. reason=201
    (NO_AP_FOUND) is a hard fail checked BEFORE any IP path, so a stale netif IP
    left from a prior same-subnet AP can't pass. Success needs Mac reachability.
    Falls back to the `ip` command for the already-associated (adopt) case where
    no fresh GOT_IP is emitted (hosted); native gets a fresh GOT_IP directly."""
    dut.send('')
    m = dut.mark()
    dut.send(f'sta {ssid} {pw}')
    t0 = time.time()
    ip = None
    while time.time() - t0 < timeout:
        chunk = dut.since(m)
        if sum('StaDisconnected' in l and 'reason=201' in l for l in chunk) >= 3:
            return None, 'NO_AP_FOUND (reason=201) — AP not visible to the DUT'
        for ln in chunk:
            ip = _ip_in(ln)
            if ip:
                break
        if ip:
            break
        time.sleep(0.3)
    if not ip:
        # adopt case (already on this AP): no fresh GOT_IP. Ask the netif.
        m2 = dut.mark()
        dut.send('ip')
        time.sleep(1.5)
        for ln in dut.since(m2):
            r = re.search(r'^ip (\d+\.\d+\.\d+\.\d+)', ln)
            if r and r.group(1) != '0.0.0.0':
                ip = r.group(1)
                break
    if not ip:
        return None, 'no GOT_IP and no ip-command address'
    if not reachable(ip, bind):
        return None, f'{ip} not reachable from {bind} (stale/unreachable)'
    return ip, ''


def run_tx(dut, mac, ip, bind, kind, dur, udp_mbps, dut_udp_bw):
    """DUT client (sender) -> Mac server (receiver). Report Mac (receiver)."""
    sargs = [mac, '-s', '-B', bind] + (['-u'] if kind == 'udp' else [])
    srv = subprocess.Popen(sargs, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(1.5)
    m = dut.mark()
    # -b unit differs by iperf-cmd version: the older (hosted) build wants a bare
    # integer Mbits, the newer (IDF wifi/iperf) build wants a "100M" suffix.
    bw = f'-u -b {dut_udp_bw} ' if kind == 'udp' else ''
    dut.send(f'iperf -c {bind} {bw}-t {dur}')
    time.sleep(dur + 4)
    dut_v = summary_mbps(dut.since(m))
    time.sleep(1)
    srv.terminate()
    try:
        mo = srv.communicate(timeout=5)[0] or ''
    except Exception:
        mo = ''
    mac_v = summary_mbps(mo.splitlines())
    # UDP receiver summary can be cut when the server is terminated; fall back
    # to the DUT-sent rate (link-limited offered rate) and flag it.
    report = mac_v if mac_v is not None else dut_v
    return {'report': report, 'dut': dut_v, 'mac': mac_v, 'recv': 'mac'}


def run_rx(dut, mac, ip, bind, kind, dur, udp_mbps):
    """Mac client (sender) -> DUT server (receiver). Report DUT (receiver)."""
    dut.send(f'iperf -s {"-u" if kind == "udp" else ""}')
    time.sleep(1.5)
    m = dut.mark()
    cargs = [mac, '-c', ip, '-B', bind, '-t', str(dur)]
    if kind == 'udp':
        cargs += ['-u', '-b', f'{udp_mbps}M']            # Mac: "100M" suffix
    try:
        r = subprocess.run(cargs, capture_output=True, text=True, timeout=dur + 15)
        mo = r.stdout + r.stderr
    except Exception as e:
        mo = f'(mac err {e})'
    time.sleep(2)
    dut_v = summary_mbps(dut.since(m))
    dut.send('iperf -a')
    time.sleep(1)
    mac_v = summary_mbps(mo.splitlines())
    return {'report': dut_v, 'dut': dut_v, 'mac': mac_v, 'recv': 'dut'}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', required=True, help='DUT console serial port')
    p.add_argument('--label', required=True, help='firmware under test, e.g. hosted | idf_iperf')
    p.add_argument('--transport', default='', help='sdio|spi_fd|spi_hd|uart|native')
    p.add_argument('--bind', default='192.168.50.22',
                   help='measuring-machine IP to bind iperf to (the iface that reaches the DUT)')
    p.add_argument('--aps', required=True, help='comma list of ssid:password')
    p.add_argument('--duration', type=int, default=15)
    p.add_argument('--udp-mbps', type=int, default=100, help='UDP offered rate (Mbits)')
    p.add_argument('--dut-udp-bw', default=None,
                   help='DUT-side iperf -b value; default = <udp-mbps> (older iperf-cmd, bare int). Pass e.g. 100M for the newer IDF iperf-cmd.')
    p.add_argument('--mac-iperf', default='/opt/homebrew/bin/iperf', help='iperf 2.x binary on this machine')
    p.add_argument('--book', default=os.path.join(HERE, 'bench_measurements.csv'))
    p.add_argument('--out', default='/tmp/hw_iperf')
    args = p.parse_args()
    dut_udp_bw = args.dut_udp_bw or str(args.udp_mbps)

    os.makedirs(args.out, exist_ok=True)
    aps = [tuple(a.split(':', 1)) for a in args.aps.split(',') if a]
    dut = DutConsole(args.port)
    results = {}

    def log(m):
        print(m, flush=True)

    log(f'==== iperf [{args.label}/{args.transport}] bind={args.bind} '
        f't={args.duration}s udp={args.udp_mbps}M ====')
    for ssid, pw in aps:
        log(f'\n-- {ssid}: associate --')
        ip, why = associate(dut, ssid, pw, args.bind)
        if not ip:
            log(f'{ssid}: SKIP — {why}')
            results[ssid] = {'err': why}
            continue
        log(f'{ssid}: DUT ip={ip}')
        r = {}
        for name, fn, kind in [('tcp_tx', run_tx, 'tcp'), ('tcp_rx', run_rx, 'tcp'),
                               ('udp_tx', run_tx, 'udp'), ('udp_rx', run_rx, 'udp')]:
            res = (run_tx(dut, args.mac_iperf, ip, args.bind, kind, args.duration, args.udp_mbps, dut_udp_bw)
                   if fn is run_tx else
                   run_rx(dut, args.mac_iperf, ip, args.bind, kind, args.duration, args.udp_mbps))
            r[name] = res['report']
            log(f'{ssid} {name}: {res["report"]} Mbps  (dut={res["dut"]} mac={res["mac"]} recv={res["recv"]})')
            time.sleep(1)
        results[ssid] = r
        bench_book.append(args.book, {
            'firmware': args.label, 'transport': args.transport, 'ssid': ssid,
            'duration_s': args.duration,
            'notes': f'iperf {args.duration}s udp_offered={args.udp_mbps}M bind={args.bind}',
            'tcp_tx_mbps': _f(r.get('tcp_tx')), 'tcp_rx_mbps': _f(r.get('tcp_rx')),
            'udp_tx_mbps': _f(r.get('udp_tx')), 'udp_rx_mbps': _f(r.get('udp_rx')),
            'artifacts_dir': args.out,
        })

    dut.close()
    log('\n==== SUMMARY (Mbps, receiver-side) ====')
    log(f'{"AP":14s} {"TCP_TX":>8s} {"TCP_RX":>8s} {"UDP_TX":>8s} {"UDP_RX":>8s}')
    for ssid, _ in aps:
        r = results.get(ssid, {})
        if 'err' in r:
            log(f'{ssid:14s} SKIP ({r["err"]})')
            continue
        g = lambda k: f'{r[k]:.1f}' if r.get(k) is not None else '-'
        log(f'{ssid:14s} {g("tcp_tx"):>8s} {g("tcp_rx"):>8s} {g("udp_tx"):>8s} {g("udp_rx"):>8s}')
    log('==== DONE ====')


def _f(v):
    return f'{v:.1f}' if v is not None else ''


if __name__ == '__main__':
    main()
