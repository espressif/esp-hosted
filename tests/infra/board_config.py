"""
Board auto-configuration — generate Kconfig overlay from hardware probe.

Two-layer overlay:
    Layer 1: Board config (auto-detected from hardware, transparent to user)
    Layer 2: User preferences (from env.json sdkconfig_optimizations)

Modes (env.json build.host_board_config / slave_board_config):
    "auto"   — probe hardware, auto-generate board Kconfig (default)
    "none"   — don't add board-specific config
    "custom" — use user-provided board_sdkconfig list from env.json

Config sources: parsed from actual Kconfig files (mcu6 + CP) at runtime.
"""

import os
from infra.kconfig_parser import get_host_choices, get_cp_choices

# Cache parsed Kconfig (populated on first call)
_host_choices_cache = None
_cp_choices_cache = None


def _get_host_choices(cfg):
    global _host_choices_cache
    if _host_choices_cache is None:
        _host_choices_cache = get_host_choices(
            getattr(cfg, 'host_base', ''))
    return _host_choices_cache


def _get_cp_choices(cfg):
    global _cp_choices_cache
    if _cp_choices_cache is None:
        _cp_choices_cache = get_cp_choices(
            getattr(cfg, 'cp_base', ''))
    return _cp_choices_cache


def _normalize_chip(chip_str):
    """Normalize 'ESP32-C6FH4' → 'ESP32C6' (matches Kconfig symbol suffix)."""
    chip = (chip_str or '').upper().replace('-', '')
    # Strip package suffix: ESP32C6FH4 → ESP32C6
    for known in ['ESP32C61', 'ESP32C6', 'ESP32C5', 'ESP32C3', 'ESP32C2',
                   'ESP32S3', 'ESP32S2', 'ESP32H2', 'ESP32H4', 'ESP32P4', 'ESP32']:
        if chip.startswith(known):
            return known
    return chip


def _chip_to_cp_target_symbol(chip_str):
    """'ESP32-C6FH4' → 'ESP_HOSTED_CP_TARGET_ESP32C6'."""
    norm = _normalize_chip(chip_str)
    return f'ESP_HOSTED_CP_TARGET_{norm}' if norm else None


def _depends_satisfied(depends_list, active_symbols):
    """Check if any dependency symbol is in active_symbols (OR logic)."""
    if not depends_list:
        return True  # no deps = always valid
    return any(s in active_symbols for s in depends_list)


# ── Public API ──────────────────────────────────────────────────────────

def get_cp_target_options(cfg):
    """Return list of CP target options from host Kconfig.

    Returns: [{'symbol': 'ESP_HOSTED_CP_TARGET_ESP32C6',
               'label': 'ESP32-C6 (fetched from ...)', 'depends': [...]}]
    """
    choices = _get_host_choices(cfg)
    target_choice = choices.get('ESP_HOSTED_CP_TARGET', {})
    return target_choice.get('options', [])


def get_board_options(cfg, slave_chip=None):
    """Return board options compatible with detected slave chip.

    Args:
        cfg: EhTestConfig
        slave_chip: detected slave chip string (e.g. 'ESP32-C6FH4')

    Returns: [{'symbol': ..., 'label': ..., 'depends': ...}]
    """
    choices = _get_host_choices(cfg)
    board_choice = choices.get('ESP_HOSTED_P4_DEV_BOARD', {})
    all_options = board_choice.get('options', [])

    if not slave_chip:
        return all_options

    # Build set of "active" symbols based on detected slave
    cp_target_sym = _chip_to_cp_target_symbol(slave_chip)
    # Also add SLAVE_IDF_TARGET_ variant (some depends use this)
    slave_idf_sym = f'SLAVE_IDF_TARGET_{_normalize_chip(slave_chip)}'
    active = {cp_target_sym, slave_idf_sym} if cp_target_sym else set()

    return [opt for opt in all_options
            if _depends_satisfied(opt['depends'], active)]


def get_transport_options(cfg):
    """Return CP transport options from CP Kconfig."""
    choices = _get_cp_choices(cfg)
    transport_choice = choices.get('EH_TRANSPORT_CP_INTERFACE', {})
    return transport_choice.get('options', [])


def get_cp_host_type_options(cfg):
    """Return CP host type options (Linux FG, NG, MCU)."""
    choices = _get_cp_choices(cfg)
    return choices.get('ESP_HOSTED_CP_TYPE', {}).get('options', [])


def get_cp_board_options(cfg, slave_chip=None):
    """Return CP-side board options, filtered by IDF target (slave chip)."""
    choices = _get_cp_choices(cfg)
    all_options = choices.get('ESP_HOST_DEV_BOARD', {}).get('options', [])

    if not slave_chip:
        return all_options

    # Build active symbols from slave chip
    norm = _normalize_chip(slave_chip)
    active = {f'IDF_TARGET_{norm}'}
    # Also add host type symbols (both for now)
    active.add('ESP_HOSTED_CP_FOR_MCU')
    active.add('ESP_HOSTED_CP_FOR_LINUX')

    return [opt for opt in all_options
            if _depends_satisfied(opt['depends'], active)]


def generate_board_sdkconfig(board_symbol, slave_chip, host_chip=None, host_rev=None):
    """
    Generate host-side Kconfig lines from selections.

    Args:
        board_symbol: selected board symbol (e.g. 'ESP_HOSTED_P4_C6_CORE_BOARD')
        slave_chip: detected slave chip (e.g. 'ESP32-C6FH4')
        host_chip: detected host chip (e.g. 'ESP32-P4')
        host_rev: silicon revision string (e.g. '0.2')

    Returns: list of CONFIG_X=y lines
    """
    lines = []

    # CP target
    cp_sym = _chip_to_cp_target_symbol(slave_chip)
    if cp_sym:
        lines.append(f'CONFIG_{cp_sym}=y')

    # Board selection
    if board_symbol:
        lines.append(f'CONFIG_{board_symbol}=y')

    # Silicon revision
    if host_chip and 'P4' in _normalize_chip(host_chip):
        if host_rev:
            try:
                if float(host_rev) < 3.0:
                    lines.append('CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y')
            except ValueError:
                pass

    return lines


def _resolve_overlay(mode, auto_lines, user_lines):
    """Resolve overlay based on mode + user overrides."""
    if mode == 'none':
        return []
    if mode == 'custom':
        return user_lines
    if not user_lines:
        return auto_lines
    user_keys = {l.split('=')[0] for l in user_lines if '=' in l}
    merged = [l for l in auto_lines if l.split('=')[0] not in user_keys]
    merged.extend(user_lines)
    return merged


def get_host_overlay(cfg, host_probe, slave_probe):
    """Board overlay for host builds."""
    mode = getattr(cfg, 'host_board_config_mode', 'auto')
    user = getattr(cfg, 'host_board_sdkconfig', [])

    if mode != 'auto':
        return _resolve_overlay(mode, [], user)

    # Auto: determine board symbol from env.json board name
    board_name = getattr(cfg, 'board', '')
    slave_chip = slave_probe.get('chip', '')
    host_chip = host_probe.get('chip', '')
    host_rev = host_probe.get('revision')

    # Map board name → symbol
    boards = get_board_options(cfg, slave_chip)
    # Try to find the board symbol matching the env.json board name
    board_symbol = _board_name_to_symbol(board_name, boards)

    auto = generate_board_sdkconfig(board_symbol, slave_chip, host_chip, host_rev)
    return _resolve_overlay('auto', auto, user)


def get_slave_overlay(cfg, host_probe, slave_probe):
    """Board overlay for slave/CP builds."""
    mode = getattr(cfg, 'slave_board_config_mode', 'auto')
    user = getattr(cfg, 'slave_board_sdkconfig', [])
    # CP auto-config is empty — CP knows its own chip
    return _resolve_overlay(mode, [], user)


def _board_name_to_symbol(board_name, board_options):
    """Map env.json board name to Kconfig symbol.

    e.g. 'p4_c6_core_board' → 'ESP_HOSTED_P4_C6_CORE_BOARD'
    """
    if not board_name:
        return None
    name_upper = board_name.upper()
    for opt in board_options:
        # Match by checking if the board name is a substring of the symbol
        sym = opt['symbol']
        if name_upper.replace('_', '') in sym.replace('_', ''):
            return sym
        # Also try direct match after uppercasing
        if f'ESP_HOSTED_{name_upper}' == sym or sym.endswith(name_upper):
            return sym
    # Fallback: convert board_name to likely symbol
    return board_name.upper()
