# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import os
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from kconfiglib.core import Kconfig

# Template for ESP-IDF-style minimal config file header (sdkconfig.defaults).
# Substitute with the actual IDF version and IDF_TARGET symbol's config_string
# (when not the default esp32).
MIN_CONFIG_HEADER_TEMPLATE = (
    "# This file was generated using idf.py save-defconfig or menuconfig [D] key. It can be edited manually.\n"
    "# Espressif IoT Development Framework (ESP-IDF) {idf_version} Project Minimal Configuration\n"
    "#\n"
    "{idf_target_config_string}"
)


def build_idf_min_config_header(config: "Kconfig", idf_version: str = "") -> str:
    """
    Build the ESP-IDF minimal-config header from ``MIN_CONFIG_HEADER_TEMPLATE``.

    Reads ``IDF_VERSION`` from the environment when *idf_version* is not given
    (or empty).  Prepends the ``IDF_TARGET`` assignment when the target is not
    the default ``esp32``.
    """

    if not idf_version:
        idf_version = os.environ.get("IDF_VERSION", "")
    idf_target_config_string = ""
    sym = config.syms.get("IDF_TARGET")
    if sym is not None and sym.str_value != "esp32":
        idf_target_config_string = sym.config_string
    return MIN_CONFIG_HEADER_TEMPLATE.format(
        idf_version=idf_version,
        idf_target_config_string=idf_target_config_string,
    )
