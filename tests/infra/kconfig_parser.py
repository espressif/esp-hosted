"""
Lightweight Kconfig parser — extract choice blocks with dependencies.

NOT a full Kconfig parser. Only extracts:
- choice blocks (symbol, options, depends)
- config entries within choices (symbol, label, depends)

Used by board_config.py to dynamically populate board/target options
from the actual mcu6 and CP Kconfig files.
"""

import os
import re
from pathlib import Path


def parse_choices(kconfig_path):
    """
    Parse all choice blocks from a Kconfig file.

    Returns: {
        'ESP_HOSTED_CP_TARGET': {
            'prompt': 'Choose the Co-processor to use',
            'depends': ['ESP_HOSTED_ENABLED'],
            'default': 'ESP_HOSTED_CP_TARGET_ESP32C6',
            'options': [
                {
                    'symbol': 'ESP_HOSTED_CP_TARGET_ESP32C6',
                    'label': 'ESP32-C6 (fetched from Wi-Fi Remote Component)',
                    'depends': ['SLAVE_IDF_TARGET_ESP32C6'],
                },
                ...
            ]
        },
        ...
    }
    """
    if not os.path.exists(kconfig_path):
        return {}

    with open(kconfig_path) as f:
        lines = f.readlines()

    choices = {}
    i = 0
    while i < len(lines):
        line = lines[i].strip()

        # Match: choice SYMBOL or choice (unnamed)
        m = re.match(r'^choice\s+(\w+)', line)
        if m:
            choice_name = m.group(1)
            choice_data = _parse_choice_block(lines, i)
            if choice_data:
                choices[choice_name] = choice_data
                # Skip past endchoice
                while i < len(lines) and 'endchoice' not in lines[i]:
                    i += 1
        i += 1

    return choices


def _parse_choice_block(lines, start_idx):
    """Parse a single choice block starting at start_idx."""
    choice = {
        'prompt': '',
        'depends': [],
        'default': '',
        'options': [],
    }

    i = start_idx + 1
    current_config = None

    while i < len(lines):
        line = lines[i].strip()

        if line == 'endchoice':
            # Flush last config
            if current_config:
                choice['options'].append(current_config)
            return choice

        # Choice-level attributes
        if current_config is None:
            m = re.match(r'^bool\s+"(.+)"', line)
            if m:
                choice['prompt'] = m.group(1)

            m = re.match(r'^depends\s+on\s+(.+)', line)
            if m:
                choice['depends'] = _parse_depends(m.group(1))

            m = re.match(r'^default\s+(\w+)', line)
            if m:
                choice['default'] = m.group(1)

        # Config entry within choice
        m = re.match(r'^config\s+(\w+)', line)
        if m:
            if current_config:
                choice['options'].append(current_config)
            current_config = {
                'symbol': m.group(1),
                'label': '',
                'depends': [],
            }
            i += 1
            continue

        # Config-level attributes
        if current_config:
            m = re.match(r'^bool\s+"(.+)"', line)
            if m:
                current_config['label'] = m.group(1)

            m = re.match(r'^depends\s+on\s+(.+)', line)
            if m:
                current_config['depends'] = _parse_depends(m.group(1))

        i += 1

    # EOF without endchoice
    if current_config:
        choice['options'].append(current_config)
    return choice


def _parse_depends(dep_str):
    """
    Extract symbol names from a depends expression.
    'ESP_HOSTED_CP_TARGET_ESP32C6 || ESP_HOSTED_CP_TARGET_ESP32C5'
    → ['ESP_HOSTED_CP_TARGET_ESP32C6', 'ESP_HOSTED_CP_TARGET_ESP32C5']
    """
    symbols = re.findall(r'\b([A-Z][A-Z0-9_]+)\b', dep_str)
    # Filter out operators that look like symbols
    return [s for s in symbols if s not in ('ON', 'OFF', 'OR', 'AND', 'NOT')]


def find_host_kconfig(host_base):
    """Find mcu6 Kconfig from host_base path."""
    # host_base is typically ~/code/esp_hosted_mcu6/examples
    # Kconfig is at ~/code/esp_hosted_mcu6/Kconfig
    parent = Path(host_base).resolve().parent
    kconfig = parent / 'Kconfig'
    if kconfig.exists():
        return str(kconfig)
    return None


def find_cp_kconfigs(cp_base):
    """Find CP Kconfig files from cp_base path."""
    # cp_base is typically <repo_root>/examples (this-repo mode) or
    # an external CP checkout dir.  Walk up to the first ancestor
    # that contains a `coprocessor/` directory.
    repo_root = Path(cp_base).resolve()
    for _ in range(6):
        cp_dir = repo_root / 'coprocessor'
        if cp_dir.exists():
            result = {}
            # Main CP Kconfig (host type, board selection)
            main = cp_dir / 'Kconfig.ext'
            if main.exists():
                result['main'] = str(main)
            # Transport Kconfig
            transport = cp_dir / 'eh_cp_transport' / 'Kconfig.ext'
            if transport.exists():
                result['transport'] = str(transport)
            return result
        repo_root = repo_root.parent
    return {}


def get_host_choices(host_base):
    """Get parsed CP target and board choices from host Kconfig."""
    kconfig = find_host_kconfig(host_base)
    if not kconfig:
        return {}
    return parse_choices(kconfig)


def get_cp_choices(cp_base):
    """Get parsed transport choices from CP Kconfig."""
    paths = find_cp_kconfigs(cp_base)
    result = {}
    for name, path in paths.items():
        result.update(parse_choices(path))
    return result
