#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
#
# Auto-generate eh_common_fw_version.h from idf_component.yml.
# Single source of truth for version — run as pre-commit hook or manually.
#
# Usage:
#   python tools/check_fw_versions.py           # check only (exit 1 if stale)
#   python tools/check_fw_versions.py -u        # update if stale
#   python tools/check_fw_versions.py -f        # force regenerate

import argparse
import os
import re
import sys
from datetime import date
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
YML_FILE = REPO_ROOT / "idf_component.yml"
FW_VERSION_HEADER = REPO_ROOT / "modules" / "common" / "eh_common" / "include" / "eh_common_fw_version.h"

START_YEAR = 2025


def get_yml_version():
    """Parse version from idf_component.yml. Returns (major, minor, patch, prerelease)."""
    text = YML_FILE.read_text()
    # Match "3.0.0" or "3.0.0-rc1" etc.
    m = re.search(r'^version:\s*"(\d+)\.(\d+)\.(\d+)(?:-([a-zA-Z0-9._-]+))?"', text, re.MULTILINE)
    if not m:
        print(f"ERROR: Could not parse version from {YML_FILE}")
        sys.exit(2)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4) or "")


def get_header_version():
    """Extract version from existing header. Returns (major, minor, patch) or None."""
    if not FW_VERSION_HEADER.exists():
        return None
    text = FW_VERSION_HEADER.read_text()
    maj = re.search(r'#define\s+PROJECT_VERSION_MAJOR_1\s+(\d+)', text)
    min_ = re.search(r'#define\s+PROJECT_VERSION_MINOR_1\s+(\d+)', text)
    pat = re.search(r'#define\s+PROJECT_VERSION_PATCH_1\s+(\d+)', text)
    if not (maj and min_ and pat):
        return None
    return (int(maj.group(1)), int(min_.group(1)), int(pat.group(1)))


def generate_header(major, minor, patch, prerelease):
    """Generate the full eh_common_fw_version.h content."""
    year = date.today().year
    if year == START_YEAR:
        year_str = str(START_YEAR)
    else:
        year_str = f"{START_YEAR}-{year}"

    pre_tag = f" ({prerelease})" if prerelease else ""

    return f"""\
/*
 * SPDX-FileCopyrightText: {year_str} Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 *
 * DO NOT MODIFY THIS FILE.
 *
 * tools/check_fw_versions.py generated this file.
 * Version info here is populated from idf_component.yml{pre_tag}
 */

#ifndef __EH_COMMON_FW_VERSION__H
#define __EH_COMMON_FW_VERSION__H

#include <stdint.h>

/*
 * eh_common_fw_version.h --- firmware version definitions (macros + wire struct)
 *
 * Provides:
 *   - Build-time version macros (PROJECT_NAME, PROJECT_VERSION_*)
 *   - The wire-shared firmware version struct broadcast by the coprocessor
 *     in the PRIV TLV handshake via ESP_PRIV_FIRMWARE_VER (type 0x17).
 *
 * The struct uses a union so both old naming (major1/major2/minor/
 * revision_patch_1/2) and new naming (major/minor/patch/revision/build) map
 * to the same 5 bytes. Old callers compile unchanged; new callers use the
 * clean names.
 *
 * Wire layout (15 bytes total):
 *   [0-9]  project_name (char[10])
 *   [10]   major  / major1
 *   [11]   minor  / major2
 *   [12]   patch  / minor_old
 *   [13]   revision / revision_patch_1
 *   [14]   build    / revision_patch_2
 */

/* -- Build-time version macros (clean naming, matches mcu6) --------------- */

#define PROJECT_NAME              "EHCP"
#define PROJECT_VERSION_MAJOR_1   {major}
#define PROJECT_VERSION_MINOR_1   {minor}
#define PROJECT_VERSION_PATCH_1   {patch}

/* Compat aliases: old code uses MAJOR_2/MINOR/REVISION_PATCH — remove after
   Linux V1 proto rename (see todo.md) */
#define PROJECT_VERSION_MAJOR_2      PROJECT_VERSION_MINOR_1
#define PROJECT_VERSION_MINOR        PROJECT_VERSION_PATCH_1
#define PROJECT_REVISION_PATCH_1     0
#define PROJECT_REVISION_PATCH_2     0

/**
 * Macro to convert version number into an integer
 */
#define EH_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))

/* Extract version components from version value */
#define EH_VERSION_MAJOR(ver) (((ver) >> 16) & 0xFF)
#define EH_VERSION_MINOR(ver) (((ver) >> 8) & 0xFF)
#define EH_VERSION_PATCH(ver) ((ver) & 0xFF)

/* Format version tuple for printing */
#define EH_VERSION_PRINTF_FMT "%u.%u.%u"
#define EH_VERSION_PRINTF_ARGS(ver) \\
    (unsigned int)EH_VERSION_MAJOR(ver), \\
    (unsigned int)EH_VERSION_MINOR(ver), \\
    (unsigned int)EH_VERSION_PATCH(ver)

/* -- Wire-shared firmware version struct ---------------------------------- */
struct eh_fw_version {{
    char project_name[10];

    union {{
        struct {{
            /* NEW naming -- recommended for new code */
            uint8_t major;
            uint8_t minor;
            uint8_t patch;
            uint8_t revision;
            uint8_t build;
        }};
        struct {{
            /* OLD naming -- backward compatible, old callers unchanged */
            uint8_t major1;
            uint8_t major2;
            uint8_t minor_old;          /* was 'minor' in old struct fw_version */
            uint8_t revision_patch_1;
            uint8_t revision_patch_2;
        }};
        uint8_t version[5];             /* raw array access */
    }};
}} __attribute__((packed));

_Static_assert(sizeof(struct eh_fw_version) == 15,
               "eh_fw_version must be exactly 15 bytes");

/* -- Aliases for backward compatibility ----------------------------------- */

/* New preferred typedef */
typedef struct eh_fw_version eh_fw_version_t;

/* Old code uses 'struct fw_version' or 'fw_version_t' -- keep both working */
struct fw_version {{
    char project_name[10];
    uint8_t major1;
    uint8_t major2;
    uint8_t minor;
    uint8_t revision_patch_1;
    uint8_t revision_patch_2;
}} __attribute__((packed));

typedef struct eh_fw_version fw_version_t;

#endif /* __EH_COMMON_FW_VERSION__H */
"""


def check_and_update(update=False, force=False):
    major, minor, patch, prerelease = get_yml_version()
    ver_str = f"{major}.{minor}.{patch}"
    if prerelease:
        ver_str += f"-{prerelease}"

    header_ver = get_header_version()
    yml_tuple = (major, minor, patch)
    stale = (header_ver != yml_tuple)

    if stale:
        print(f"Version mismatch: idf_component.yml={ver_str}, "
              f"header={header_ver or 'MISSING'}")

    if force or (stale and update):
        content = generate_header(major, minor, patch, prerelease)
        FW_VERSION_HEADER.parent.mkdir(parents=True, exist_ok=True)
        FW_VERSION_HEADER.write_text(content)
        action = "Force-updated" if force else "Updated"
        print(f"{action} {FW_VERSION_HEADER.relative_to(REPO_ROOT)} -> {ver_str}")
        return 0

    if stale:
        print(f"Run with -u to update or -f to force regenerate.")
        return 1

    print(f"Version OK: {ver_str}")
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Check/update eh_common_fw_version.h against idf_component.yml")
    parser.add_argument('-u', '--update', action='store_true',
                        help="Update header if version differs")
    parser.add_argument('-f', '--force', action='store_true',
                        help="Force regenerate header regardless of match")
    args = parser.parse_args()
    sys.exit(check_and_update(update=args.update, force=args.force))
