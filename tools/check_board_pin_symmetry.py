#!/usr/bin/env python3
"""Every transport on a board must reuse that board's SDIO wires.

Boards route six lines between host and coprocessor for SDIO - CMD, CLK, D0..D3.
Every other transport is then wired over those SAME six wires, so that an
existing SDIO-capable board can be used to bring up spi_fd, spi_hd or uart with
no rework. The mapping is positional and identical on both sides:

    SDIO   | spi_fd      | spi_hd      | uart
    -------+-------------+-------------+---------------
    CMD    | CS          | CS          | -
    CLK    | CLK         | CLK         | -
    D0     | MOSI        | D0          | host TX -> cp RX
    D1     | MISO        | D1          | cp TX -> host RX
    D2     | Handshake   | D2          | -
    D3     | Data Ready  | D3 / DR     | -

So a transport's pin defaults are not free choices: they are derivable from the
SDIO defaults of the same side and the same board. That makes this checkable
without a schematic, and it is the only place the convention is written down -
2.x documents "use IO_MUX pins" and publishes separate host/coprocessor tables,
but never states the reuse rule, and there is no UART table at all.

What it catches, from real bugs:
  * ESP_HOSTED_P4_C5_CORE_BOARD had uart TX/RX on the HOST side (P4 49/48 = its
    D0/D1) and NOTHING on the coprocessor side, so a C5 fell through to its
    bare-target default 14/13 - which on that board are D2/D3, the handshake and
    data-ready wires. The two ends talked on different pairs and the host
    boot-looped 18 times in esp_wifi_init(). No emulator test can see this: the
    emulator connects the two firmwares over a socket, so pin numbers never
    participate, and it overlays a different board than the bench anyway.

Usage:  python3 tools/check_board_pin_symmetry.py [--verbose] [--board B]
Exit:   0 = consistent, 1 = a transport deviates from the board's SDIO wires.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HOST_DIR = os.path.join(ROOT, 'host', 'mcu', 'eh_host_mcu_transport')
CP_DIR = os.path.join(ROOT, 'coprocessor', 'eh_cp_transport')

# side -> the SDIO option carrying each wire
SDIO = {
    'host': {w: f'ESP_HOSTED_HOST_SDIO_PIN_{w}' for w in ('CMD', 'CLK', 'D0', 'D1', 'D2', 'D3')},
    'cp':   {w: f'EH_TRANSPORT_CP_SDIO_PIN_{w}' for w in ('CMD', 'CLK', 'D0', 'D1', 'D2', 'D3')},
}

# transport -> [(host option, coprocessor option)] that must land on the SAME
# board wire. Deliberately NOT "this signal must be D0": boards differ - the C5
# core board puts uart on D0/D1, the C6 boards put it on D3/D2 - and either is
# fine. What is never fine is the two ends choosing DIFFERENT wires, which is
# exactly the bug this exists to catch.
PAIRS = {
    'spi': [
        ('ESP_HOSTED_HOST_SPI_CS_GPIO',      'EH_TRANSPORT_CP_SPI_GPIO_CS'),
        ('ESP_HOSTED_HOST_SPI_CLK_GPIO',     'EH_TRANSPORT_CP_SPI_GPIO_CLK'),
        ('ESP_HOSTED_HOST_SPI_MOSI_GPIO',    'EH_TRANSPORT_CP_SPI_GPIO_MOSI'),
        ('ESP_HOSTED_HOST_SPI_MISO_GPIO',    'EH_TRANSPORT_CP_SPI_GPIO_MISO'),
        ('ESP_HOSTED_HOST_HANDSHAKE_GPIO',   'EH_TRANSPORT_CP_SPI_GPIO_HANDSHAKE'),
        ('ESP_HOSTED_HOST_DATA_READY_GPIO',  'EH_TRANSPORT_CP_SPI_GPIO_DATA_READY'),
    ],
    'uart': [
        # TX faces RX: the wire the host drives is the wire the coprocessor reads.
        ('ESP_HOSTED_HOST_UART_TX_GPIO',     'EH_TRANSPORT_CP_UART_PIN_RX'),
        ('ESP_HOSTED_HOST_UART_RX_GPIO',     'EH_TRANSPORT_CP_UART_PIN_TX'),
    ],
}

# The coprocessor's SDIO pins are fixed by the SoC, so its Kconfig gates them on
# IDF_TARGET, not on the board. To check a board's coprocessor side we therefore
# need to know which part sits on it. Encoded here because nothing in the Kconfig
# states it: the board choice lives on both sides independently.
BOARD_CP_TARGET = {
    'ESP_HOSTED_P4_C5_CORE_BOARD':          'ESP32C5',
    'ESP_HOSTED_P4X_C5_DEV_BOARD_FUNC_BOARD': 'ESP32C5',
    'ESP_HOSTED_C2_C5_MODULE_SUB_BOARD':    'ESP32C5',
    'ESP_ESP32C5_SPI_RASPI_MATING_BOARD':   'ESP32C5',
    'ESP_HOSTED_P4_C6_CORE_BOARD':          'ESP32C6',
    'ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD':   'ESP32C6',
    'ESP32P4_EYE_C6_BOARD':                 'ESP32C6',
    'ESP32P4_TAB5_C6_BOARD':                'ESP32C6',
    'ESP_HOSTED_P4_C61_CORE_BOARD':         'ESP32C61',
}

# Correct by construction; each needs a reason verified in the Kconfig itself.
EXEMPT = {
    ('spi', 'ESP_ESP32C5_SPI_RASPI_MATING_BOARD', 'host'):
        'host is a Raspberry Pi (HOST_TYPE_POSIX), not an MCU',
}

FILES = {
    ('host', 'sdio'): 'Kconfig.host.sdio', ('host', 'spi'): 'Kconfig.host.spi',
    ('host', 'spi_hd'): 'Kconfig.host.spi_hd', ('host', 'uart'): 'Kconfig.host.uart',
    ('cp', 'sdio'): 'Kconfig.cp.sdio', ('cp', 'spi'): 'Kconfig.cp.spi',
    ('cp', 'spi_hd'): 'Kconfig.cp.spi_hd', ('cp', 'uart'): 'Kconfig.cp.uart',
}

CONFIG_RE = re.compile(r'^\s*config\s+(\w+)')
TARGET_RE = re.compile(r'\bIDF_TARGET_(\w+)\b')
DEFAULT_RE = re.compile(r'^\s*default\s+(\S+)\s+if\s+(.+?)\s*$')
BOARD_RE = re.compile(r'\b((?:ESP_HOSTED_|ESP32)\w*BOARD\w*)\b')


def target_defaults(side, transport):
    """{option: {target: value}} from `default <v> if IDF_TARGET_x` lines."""
    path = os.path.join(HOST_DIR if side == 'host' else CP_DIR, FILES[(side, transport)])
    out, opt = {}, None
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8'):
        m = CONFIG_RE.match(line)
        if m:
            opt = m.group(1)
            continue
        d = DEFAULT_RE.match(line)
        if not d or not opt:
            continue
        cond = d.group(2)
        if BOARD_RE.search(cond):
            continue          # board-gated: handled by board_defaults()
        for t in TARGET_RE.findall(cond):
            out.setdefault(opt, {}).setdefault(t, d.group(1))
    return out


def board_defaults(side, transport):
    """{option: {board: value}} from `default <v> if <...BOARD...>` lines."""
    path = os.path.join(HOST_DIR if side == 'host' else CP_DIR, FILES[(side, transport)])
    out, opt = {}, None
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8'):
        m = CONFIG_RE.match(line)
        if m:
            opt = m.group(1)
            continue
        d = DEFAULT_RE.match(line)
        if not d or not opt:
            continue
        for b in BOARD_RE.findall(d.group(2)):
            out.setdefault(opt, {}).setdefault(b, d.group(1))
    return out


def wire_of(pin, wires, board):
    """Which board wire carries this pin, per that side's SDIO set."""
    for w, per_board in wires.items():
        if per_board.get(board) == pin:
            return w
    return None


def main():
    verbose = '--verbose' in sys.argv
    only = sys.argv[sys.argv.index('--board') + 1] if '--board' in sys.argv else None

    hs = board_defaults('host', 'sdio')
    host_wires = {w: hs.get(opt, {}) for w, opt in SDIO['host'].items()}
    boards = sorted({b for m in host_wires.values() for b in m})

    ct = target_defaults('cp', 'sdio')
    cp_wires = {}
    for w, opt in SDIO['cp'].items():
        cp_wires[w] = {b: ct.get(opt, {}).get(BOARD_CP_TARGET[b])
                       for b in boards if b in BOARD_CP_TARGET}
        cp_wires[w] = {b: v for b, v in cp_wires[w].items() if v is not None}

    problems, checked = [], 0
    for transport, pairs in sorted(PAIRS.items()):
        hd = board_defaults('host', transport)
        cd = board_defaults('cp', transport)
        ctd = target_defaults('cp', transport)
        for board in boards:
            if only and board != only:
                continue
            if board not in BOARD_CP_TARGET:
                continue
            tgt = BOARD_CP_TARGET[board]
            for hopt, copt in pairs:
                hpin = hd.get(hopt, {}).get(board)
                cpin = cd.get(copt, {}).get(board) or ctd.get(copt, {}).get(tgt)
                why = EXEMPT.get((transport, board, 'host')) or EXEMPT.get((transport, board, 'cp'))
                if why:
                    if verbose:
                        print(f'  exempt {transport}/{board}: {why}')
                    continue
                if hpin is None or cpin is None:
                    continue      # this board does not configure that signal
                hw = wire_of(hpin, host_wires, board)
                cw = wire_of(cpin, cp_wires, board)
                if hw is None or cw is None:
                    problems.append(
                        f'{transport:6s} {board}: {hopt}={hpin} (wire {hw or "NOT an SDIO wire"}) '
                        f'vs {copt}={cpin} (wire {cw or "NOT an SDIO wire"}) - every transport '
                        f'must reuse the board\'s SDIO wires')
                elif hw != cw:
                    problems.append(
                        f'{transport:6s} {board}: host {hopt}={hpin} is on {hw}, but '
                        f'coprocessor {copt}={cpin} is on {cw} - the two ends are wired '
                        f'to DIFFERENT lines and cannot talk')
                else:
                    checked += 1
                    if verbose:
                        print(f'  ok {transport}/{board}: {hw}  host {hpin} <-> cp {cpin}')

    if problems:
        print('Transport pins do not agree with the board\'s SDIO wiring:')
        for p in problems:
            print('  - ' + p)
        print(f'\n{checked} pairing(s) consistent. Board-specific defaults must sit ABOVE '
              'target-generic ones - the first matching default wins.')
        return 1
    print(f'OK: {checked} host<->coprocessor pin pairing(s) agree on the same board wire.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
