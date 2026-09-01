"""
Hardware probe — detect connected devices before testing.

Usage:
    from infra.hardware import eh_test_hw_probe_device, eh_test_hw_probe_setup

    result = eh_test_hw_probe_device('/dev/cu.usbserial-120', expected_chip='ESP32-P4')
    if not result['connected']:
        print(f"Device not found: {result['error']}")
"""

import os
import subprocess
import re
import sys


_KNOWN_TARGETS = ('esp32c61', 'esp32c6', 'esp32c5', 'esp32c3', 'esp32c2',
                  'esp32s3', 'esp32s2', 'esp32h2', 'esp32p4', 'esp32')


def chip_to_idf_target(chip_str):
    """'ESP32-C6FH4' -> 'esp32c6'; None if unrecognised. The one place that maps a
    probed chip name to an IDF target, so no caller assumes which chip is present."""
    c = (chip_str or '').upper().replace('-', '')
    for t in _KNOWN_TARGETS:
        if c.startswith(t.upper()):
            return t
    return None


def eh_test_hw_probe_device(port, expected_chip=None, timeout=5):
    """
    Probe a serial port for an ESP device.

    Returns:
        {
            'connected': bool,
            'chip': str or None,       # e.g., 'ESP32-P4', 'ESP32-C6'
            'revision': str or None,   # e.g., '0.2', '3.0'
            'port': str,
            'error': str or None
        }
    """
    result = {
        'connected': False,
        'chip': None,
        'revision': None,
        'port': port,
        'error': None,
    }

    # Check port exists
    if not os.path.exists(port):
        result['error'] = f'Port {port} does not exist'
        return result

    # Try esptool read_mac (stable across IDF versions)
    try:
        idf_python = os.path.join(os.environ.get('IDF_PYTHON_ENV_PATH', ''), 'bin', 'python')
        if not os.path.exists(idf_python):
            idf_python = 'python'
        cmd = (f'{idf_python} -m esptool --port {port}'
               f' --before default_reset --after no_reset read_mac')
        r = subprocess.run(
            cmd, shell=True,
            capture_output=True, text=True, timeout=timeout
        )
        output = r.stdout + r.stderr

        # Parse chip type. esptool <5 prints "Chip is <name>"; esptool 5 (IDF 6.0)
        # prints "Chip type:   <name>". Both fall back to "Detecting chip type...".
        chip_match = re.search(r'Chip is (\S+)', output)
        if not chip_match:
            chip_match = re.search(r'Chip type:\s+(\S+)', output)
        if not chip_match:
            chip_match = re.search(r'Detecting chip type[.\s]*(\S+)', output)

        if chip_match:
            result['chip'] = chip_match.group(1)
            result['connected'] = True

            # Extract silicon revision: "Chip is ESP32-P4 (revision v0.2)" or,
            # on esptool 5 (IDF 6.0), "Chip type:   ESP32-P4 (revision v1.3)".
            # Without the second spelling a P4 bench aborts every HW run:
            # _rev_overlay() treats an unreadable P4 revision as build-critical.
            rev_match = re.search(
                r'(?:Chip is|Chip type:)\s+\S+\s+\(revision\s+v?(\d+\.\d+)\)', output)
            if rev_match:
                result['revision'] = rev_match.group(1)

        elif 'Failed to connect' in output:
            result['error'] = 'Device not responding (check boot mode / connections)'
        else:
            result['error'] = f'Unexpected esptool output: {output[:200]}'

    except subprocess.TimeoutExpired:
        result['error'] = f'Timeout ({timeout}s) — device not responding'
    except FileNotFoundError:
        result['error'] = 'esptool not found (is IDF environment sourced?)'
    except Exception as e:
        result['error'] = str(e)

    # Verify chip matches expected
    if result['connected'] and expected_chip:
        if expected_chip.lower() not in (result['chip'] or '').lower():
            result['error'] = (
                f"Chip mismatch: expected {expected_chip}, "
                f"got {result['chip']}"
            )
            result['connected'] = False

    return result


def eh_test_hw_probe_setup(host_port, slave_port, host_chip=None, slave_chip=None):
    """
    Probe both devices.

    Returns:
        (ok: bool, message: str, host_result: dict, slave_result: dict)
    """
    print(f'  Probing host on {host_port}...')
    host = eh_test_hw_probe_device(host_port, expected_chip=host_chip)

    print(f'  Probing slave on {slave_port}...')
    slave = eh_test_hw_probe_device(slave_port, expected_chip=slave_chip)

    if host['connected'] and slave['connected']:
        msg = f"Host: {host['chip']} on {host_port} | Slave: {slave['chip']} on {slave_port}"
        return True, msg, host, slave
    else:
        errors = []
        if not host['connected']:
            errors.append(f"Host ({host_port}): {host['error']}")
        if not slave['connected']:
            errors.append(f"Slave ({slave_port}): {slave['error']}")
        return False, ' | '.join(errors), host, slave
