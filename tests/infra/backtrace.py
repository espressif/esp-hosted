"""
Backtrace decoder for ESP crash logs.

Parses Guru Meditation Error output, extracts addresses,
decodes them using addr2line, and appends a decoded section
at the end of the log.
"""

import os
import re
import subprocess


def _find_addr2line(chip):
    """Find the addr2line binary for the given chip."""
    if chip in ('esp32', 'esp32s2', 'esp32s3'):
        tool = 'xtensa-esp32-elf-addr2line'
        if chip != 'esp32':
            tool = f'xtensa-{chip}-elf-addr2line'
    else:
        tool = 'riscv32-esp-elf-addr2line'

    try:
        r = subprocess.run(['which', tool], capture_output=True, text=True, timeout=5)
        if r.returncode == 0:
            return r.stdout.strip()
    except Exception:
        pass
    return None


def _find_elf(build_dir):
    """Find the ELF file in a build directory."""
    if not os.path.isdir(build_dir):
        return None
    for f in os.listdir(build_dir):
        if f.endswith('.elf'):
            return os.path.join(build_dir, f)
    return None


def _decode_addrs(addr2line, elf, addrs):
    """Decode addresses to short function+file:line. Returns {addr: short_str}."""
    if not addrs:
        return {}
    cmd = [addr2line, '-e', elf, '-pfiaC'] + list(addrs)
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if r.returncode != 0:
            return {}
    except Exception:
        return {}

    results = {}
    for line in r.stdout.strip().split('\n'):
        if not line.strip():
            continue
        m = re.match(r'(0x[0-9a-fA-F]+):\s+(.*)', line)
        if m:
            addr = m.group(1)
            detail = m.group(2)
            if '??' in detail:
                continue
            # Shorten paths to filename only
            detail = re.sub(r'/[^\s:]+/([^/\s:]+:\d+)', r'\1', detail)
            detail = detail.replace(' (inlined by) ', ' <- ')
            results[addr] = detail
    return results


def eh_test_decode_backtrace(pytest_output, cp_build_dir, host_build_dir,
                              cp_chip='esp32c6', host_chip='esp32p4'):
    """
    Scan pytest output for crash backtraces, decode addresses,
    and append a decoded section at end of log.

    Returns (modified_output, summary_lines).
    """
    code_addr_pattern = re.compile(r'0x4[0-2][0-9a-fA-F]{6}')
    dut_pattern = re.compile(r'\[dut-(\d)\]')

    # Find which DUTs have crashes
    crash_duts = set()
    for m in re.finditer(r'\[dut-(\d)\].*Guru Meditation', pytest_output):
        crash_duts.add(int(m.group(1)))

    if not crash_duts:
        return pytest_output, []

    all_sections = []
    summaries = []

    for dut_idx in sorted(crash_duts):
        if dut_idx == 0:
            build_dir, chip, dut_name = host_build_dir, host_chip, 'Host'
        else:
            build_dir, chip, dut_name = cp_build_dir, cp_chip, 'CP'

        elf = _find_elf(build_dir)
        addr2line = _find_addr2line(chip)
        if not elf or not addr2line:
            msg = f'[{dut_name}] Crash detected (no ELF/addr2line for decode)'
            summaries.append(msg)
            all_sections.append(msg)
            continue

        # Collect all code addresses from this DUT's crash region
        addrs = set()
        in_crash = False
        for line in pytest_output.split('\n'):
            dm = dut_pattern.search(line)
            if dm and int(dm.group(1)) == dut_idx:
                if 'Guru Meditation' in line:
                    in_crash = True
                if in_crash:
                    addrs.update(code_addr_pattern.findall(line))

        decoded = _decode_addrs(addr2line, elf, sorted(addrs))

        # Build decoded section
        mepc_m = re.search(
            r'\[dut-' + str(dut_idx) + r'\].*MEPC\s*:\s*(0x[0-9a-fA-F]+)\s+RA\s*:\s*(0x[0-9a-fA-F]+)',
            pytest_output)
        if not mepc_m:
            continue

        mepc, ra = mepc_m.group(1), mepc_m.group(2)
        lines = [f'--- [{dut_name}] Crash decode ---']
        if mepc == '0x00000000':
            lines.append(f'  MEPC: NULL function pointer call')
        else:
            lines.append(f'  MEPC: {decoded.get(mepc, mepc)}')
        lines.append(f'  RA:   {decoded.get(ra, ra)}')

        # Register decode
        reg_decoded = [(a, d) for a, d in sorted(decoded.items())
                       if a not in (mepc, ra)]
        if reg_decoded:
            lines.append(f'  Resolved addresses:')
            for addr, loc in reg_decoded:
                lines.append(f'    {addr}: {loc}')

        all_sections.extend(lines)
        summaries.extend(lines)

    # Append decoded section at end of log (don't touch original output)
    modified = pytest_output
    if all_sections:
        separator = '=' * 60
        modified += (
            f'\n\n{separator}\n'
            f'  DECODED CRASH BACKTRACE\n'
            f'{separator}\n'
            + '\n'.join(all_sections) + '\n'
            + separator + '\n'
        )

    # Make DUT labels human-readable in saved logs
    modified = (modified
                .replace('[dut-0]', '[host]')
                .replace('[dut-1]', '[cp]'))

    return modified, summaries
