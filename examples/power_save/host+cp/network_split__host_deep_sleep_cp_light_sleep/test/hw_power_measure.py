#!/usr/bin/env python3
"""
Measure current for a fixed duration and append one bookkeeping row.

Used for like-for-like comparisons: run it against IDF's examples/wifi/power_save
and then against this repo's hosted coprocessor with the SAME Wi-Fi power-save
config, so the delta is attributable to hosted rather than to a config difference.

Sporadic-wake detection: the parked floor is a few mA, and every radio wake shows
as a window whose ON-DEVICE max jumps well above it. Counting those windows says
whether the coprocessor is waking only at DTIM cadence or being disturbed.

  --duration 600 --label idf_power_save --cp-sdkconfig <path>
"""
import argparse, io, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_book


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=600.0)
    p.add_argument('--label', required=True, help='firmware under test, e.g. idf_power_save | hosted')
    p.add_argument('--transport', default='')
    p.add_argument('--cp-sdkconfig', default='')
    p.add_argument('--host-sdkconfig', default='')
    p.add_argument('--spike-mA', type=float, default=300.0)
    p.add_argument('--wake-floor-mA', type=float, default=20.0,
                   help='per-window max above this counts as a radio wake')
    p.add_argument('--out', default='/tmp/hw_power')
    p.add_argument('--book', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bench_measurements.csv'))
    p.add_argument('--notes', default='')
    p.add_argument('--ssid', default='', help='AP actually associated to; a hosted host takes\n'
                                              'its credentials from the console, not Kconfig')
    a = p.parse_args()

    os.makedirs(a.out, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    csv_path = f'{a.out}/{a.label}_{stamp}.csv'

    from joulescope import scan
    devs = scan()
    if not devs:
        print('no Joulescope attached - cannot measure'); return 2
    d = devs[0]
    rows = []
    d.open()
    try:
        d.parameter_set('i_range', 'auto')
        d.parameter_set('v_range', '15V')
        f = io.open(csv_path, 'w')
        f.write('t_wall,i_mean_A,i_min_A,i_max_A,v_mean_V,charge_C,energy_J\n')

        def on_stats(st):
            sig = st.get('signals', {}); acc = st.get('accumulators', {})
            g = lambda k, fld: sig.get(k, {}).get(fld, {}).get('value', float('nan'))
            rec = (time.time(), g('current', 'µ'), g('current', 'min'), g('current', 'max'),
                   g('voltage', 'µ'), acc.get('charge', {}).get('value', float('nan')),
                   acc.get('energy', {}).get('value', float('nan')))
            rows.append(rec)
            f.write('%.3f,%.6f,%.6f,%.6f,%.4f,%.6f,%.6f\n' % rec); f.flush()

        d.statistics_callback_register(on_stats, 'sensor')
        d.start()
        print(f'measuring {a.label} for {a.duration:.0f}s ...', flush=True)
        t0 = time.time()
        while time.time() - t0 < a.duration:
            time.sleep(5)
            if int(time.time() - t0) % 60 < 5 and rows:
                mA = 1000 * sum(r[1] for r in rows) / len(rows)
                print(f'  t={time.time()-t0:5.0f}s  running avg={mA:7.3f} mA  '
                      f'windows={len(rows)}', flush=True)
        d.stop()
        f.close()
    finally:
        # leave the current path conducting: on a JS110 this instrument IS the
        # target's supply, and closing with the range open powers the board down.
        try: d.parameter_set('i_range', 'auto')
        except Exception: pass
        d.close()

    if not rows:
        print('no samples captured'); return 2
    means = [r[1] for r in rows]; maxes = [r[3] for r in rows]; mins = [r[2] for r in rows]
    ms = sorted(maxes)
    pct = lambda q: 1000 * ms[min(int(q * len(ms)), len(ms) - 1)]
    spike = [i for i, m in enumerate(maxes) if m >= a.spike_mA / 1000.0]
    wake = [m for m in maxes if m >= a.wake_floor_mA / 1000.0]
    avg = 1000 * sum(means) / len(means)
    avg_excl = (1000 * sum(means[i] for i in range(len(means)) if i not in set(spike))
                / max(len(means) - len(spike), 1))
    charge = rows[-1][5] - rows[0][5]
    energy = rows[-1][6] - rows[0][6]

    res = dict(
        firmware=a.label, transport=a.transport, notes=a.notes,
        duration_s=f'{a.duration:.0f}', phase='steady',
        avg_mA=f'{avg:.3f}', min_mA=f'{1000*min(mins):.3f}', max_mA=f'{1000*max(maxes):.3f}',
        p50_max_mA=f'{pct(.5):.1f}', p90_max_mA=f'{pct(.9):.1f}', p99_max_mA=f'{pct(.99):.1f}',
        spike_floor_mA=f'{a.spike_mA:.0f}', spike_windows=str(len(spike)),
        spike_pct=f'{100.0*len(spike)/len(maxes):.2f}', avg_excl_spikes_mA=f'{avg_excl:.3f}',
        charge_C=f'{charge:.4f}', energy_J=f'{energy:.4f}',
        voltage_V=f'{sum(r[4] for r in rows)/len(rows):.3f}',
        sporadic_wakes=str(len(wake)), wake_floor_mA=f'{a.wake_floor_mA:.0f}',
        samples_csv=csv_path, artifacts_dir=a.out,
    )
    res.update(bench_book.row_from_config(a.cp_sdkconfig or None, a.host_sdkconfig or None))
    if a.ssid:
        res['ssid'] = a.ssid
    bench_book.append(a.book, res)

    print('\n==== %s: %.0fs ====' % (a.label, a.duration))
    print(f'  average           {avg:8.3f} mA   ({res["voltage_V"]} V)')
    print(f'  min / max         {1000*min(mins):8.3f} / {1000*max(maxes):.3f} mA')
    print(f'  per-window max    p50={pct(.5):.1f}  p90={pct(.9):.1f}  p99={pct(.99):.1f} mA')
    print(f'  >= {a.spike_mA:.0f} mA         {len(spike)} windows ({res["spike_pct"]}%)  '
          f'avg without them {avg_excl:.3f} mA')
    print(f'  radio wakes (>= {a.wake_floor_mA:.0f} mA)  {len(wake)} of {len(maxes)} windows '
          f'({100.0*len(wake)/len(maxes):.1f}%)')
    print(f'  charge / energy   {charge:.4f} C / {energy:.4f} J')
    print(f'  config            ps={res.get("ps_mode","?")} listen={res.get("listen_interval","?")} '
          f'cpu_max={res.get("cpu_max_mhz","?")}MHz tickless={res.get("tickless_idle","?")}')
    print(f'  book              {a.book}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
