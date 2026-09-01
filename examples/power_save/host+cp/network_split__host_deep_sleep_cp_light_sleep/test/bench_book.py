#!/usr/bin/env python3
"""
Bench bookkeeping: one CSV row per measurement, importable straight into Sheets.

Why a CSV and not a formatted workbook: every column here is either read out of
an sdkconfig or computed from the capture, so the file is regenerable and diffable,
and Sheets/Excel import it without a converter. One row per run, append-only -
never rewritten - so the history of what was measured under which config survives.

Config is READ FROM THE BUILD, not typed in: a hand-copied "ps_mode" column is
the first thing to go stale and then quietly invalidate a comparison.
"""
import csv, io, os, re, time

COLUMNS = [
    # identity
    'run_id', 'utc', 'local_time', 'firmware', 'transport', 'notes',
    # hardware
    'host_target', 'cp_target', 'board', 'cp_rev', 'idf_version',
    # wifi / power-save config (read from sdkconfig)
    'ps_mode', 'listen_interval', 'beacon_timeout', 'ssid',
    'cpu_max_mhz', 'pm_enable', 'tickless_idle', 'slp_disable_gpio',
    'pwrdown_periph_in_ls', 'wifi_slp_iram_opt', 'freertos_hz',
    # hosted-only config
    'unload_bus_while_sleeping', 'deassert_hs_on_cs', 'cp_light_sleep',
    'wake_host_on_cp_boot', 'host_wake_gpio', 'cp_wake_gpio',
    # what was run
    'duration_s', 'cycles', 'dwell_s', 'phase',
    # power results
    'avg_mA', 'min_mA', 'max_mA', 'p50_max_mA', 'p90_max_mA', 'p99_max_mA',
    'spike_floor_mA', 'spike_windows', 'spike_pct', 'avg_excl_spikes_mA',
    'charge_C', 'energy_J', 'voltage_V',
    # throughput results (iperf, Mbps; receiver-side)
    'tcp_tx_mbps', 'tcp_rx_mbps', 'udp_tx_mbps', 'udp_rx_mbps',
    # behavioural observations
    'sporadic_wakes', 'wake_floor_mA', 'cycles_ok', 'cycles_failed',
    'noresp', 'abort', 'guru', 'cp_counter_reset', 'cp_counters', 'host_counters',
    # provenance
    'samples_csv', 'jls_files', 'artifacts_dir',
]


def sdkconfig(path):
    """Parse an sdkconfig into a dict; missing file -> {}."""
    out = {}
    if not path or not os.path.exists(path):
        return out
    for line in io.open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        k, _, v = line.partition('=')
        out[k.strip()] = v.strip().strip('"')
    return out


def ps_mode_of(cfg):
    """Resolve the effective Wi-Fi PS mode from either example's Kconfig style."""
    if cfg.get('CONFIG_EXAMPLE_POWER_SAVE_MIN_MODEM') == 'y' or \
       cfg.get('CONFIG_APP_WIFI_PS_MIN_MODEM') == 'y':
        return 'WIFI_PS_MIN_MODEM'
    if cfg.get('CONFIG_EXAMPLE_POWER_SAVE_MAX_MODEM') == 'y' or \
       cfg.get('CONFIG_APP_WIFI_PS_MAX_MODEM') == 'y':
        return 'WIFI_PS_MAX_MODEM'
    if cfg.get('CONFIG_EXAMPLE_POWER_SAVE_NONE') == 'y' or \
       cfg.get('CONFIG_APP_WIFI_PS_NONE') == 'y':
        return 'WIFI_PS_NONE'
    return ''


def cpu_max_of(cfg):
    for mhz in ('240', '160', '120', '80', '40'):
        if cfg.get(f'CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_{mhz}') == 'y':
            return mhz
    return cfg.get('CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ', '')


def row_from_config(cp_sdk=None, host_sdk=None):
    """Build the config half of a row from the actual build outputs."""
    c, h = sdkconfig(cp_sdk), sdkconfig(host_sdk)
    yn = lambda d, k: 'y' if d.get(k) == 'y' else 'n'
    # PS mode / SSID live on whichever side drives Wi-Fi: the host for hosted,
    # the CP itself for a standalone IDF example.
    drv = h if h else c
    return {
        'cp_target': c.get('CONFIG_IDF_TARGET', ''),
        'host_target': h.get('CONFIG_IDF_TARGET', ''),
        'board': 'P4_C5_CORE_BOARD' if (c.get('CONFIG_ESP_HOSTED_P4_C5_CORE_BOARD') == 'y'
                                        or h.get('CONFIG_ESP_HOSTED_P4_C5_CORE_BOARD') == 'y') else '',
        'ps_mode': ps_mode_of(drv),
        'listen_interval': drv.get('CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL',
                                   drv.get('CONFIG_APP_WIFI_LISTEN_INTERVAL', '')),
        'beacon_timeout': drv.get('CONFIG_EXAMPLE_WIFI_BEACON_TIMEOUT', ''),
        'ssid': drv.get('CONFIG_EXAMPLE_WIFI_SSID',
                        drv.get('CONFIG_APP_WIFI_SSID', '')),
        'cpu_max_mhz': cpu_max_of(c or h),
        'pm_enable': yn(c or h, 'CONFIG_PM_ENABLE'),
        'tickless_idle': yn(c or h, 'CONFIG_FREERTOS_USE_TICKLESS_IDLE'),
        'slp_disable_gpio': yn(c or h, 'CONFIG_PM_SLP_DISABLE_GPIO'),
        'pwrdown_periph_in_ls': yn(c or h, 'CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP'),
        'wifi_slp_iram_opt': yn(c or h, 'CONFIG_ESP_WIFI_SLP_IRAM_OPT'),
        'freertos_hz': (c or h).get('CONFIG_FREERTOS_HZ', ''),
        'unload_bus_while_sleeping': yn(c, 'CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING'),
        'deassert_hs_on_cs': yn(c, 'CONFIG_EH_TRANSPORT_CP_SPI_DEASSERT_HS_ON_CS'),
        'cp_light_sleep': yn(c, 'CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_ENABLE'),
        'wake_host_on_cp_boot': yn(c, 'CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_WAKE_HOST_ON_CP_BOOT'),
        'cp_wake_gpio': c.get('CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO', ''),
        'host_wake_gpio': h.get('CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO', ''),
        'idf_version': io.open(os.path.expanduser('~/esp-idf/version.txt')).read().strip()
                       if os.path.exists(os.path.expanduser('~/esp-idf/version.txt')) else '',
    }


def append(book_csv, row):
    """Append one row, writing the header on first use."""
    new = not os.path.exists(book_csv)
    os.makedirs(os.path.dirname(book_csv) or '.', exist_ok=True)
    full = {k: '' for k in COLUMNS}
    full.update({k: v for k, v in row.items() if k in COLUMNS})
    extra = [k for k in row if k not in COLUMNS]
    if extra:
        full['notes'] = (full['notes'] + ' | ' if full['notes'] else '') + \
                        ' '.join(f'{k}={row[k]}' for k in extra)
    full.setdefault('run_id', str(int(time.time())))
    full['utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    full['local_time'] = time.strftime('%Y-%m-%d %H:%M:%S')
    with io.open(book_csv, 'a', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            w.writeheader()
        w.writerow(full)
    return book_csv
