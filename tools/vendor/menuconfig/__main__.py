# SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import sys

from .core import _main

if __name__ == "__main__":
    try:
        _main()
    except Exception as e:
        print(f"A fatal error occurred: {e}", file=sys.stderr)
        sys.exit(2)
