/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 *
 * DO NOT MODIFY THIS FILE.
 *
 * tools/check_fw_versions.py generated this file.
 * Version info here is populated from idf_component.yml (rc1)
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
#define PROJECT_VERSION_MAJOR_1   3
#define PROJECT_VERSION_MINOR_1   0
#define PROJECT_VERSION_PATCH_1   0

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
#define EH_VERSION_PRINTF_ARGS(ver) \
    (unsigned int)EH_VERSION_MAJOR(ver), \
    (unsigned int)EH_VERSION_MINOR(ver), \
    (unsigned int)EH_VERSION_PATCH(ver)

/* -- Wire-shared firmware version struct ---------------------------------- */
struct eh_fw_version {
    char project_name[10];

    union {
        struct {
            /* NEW naming -- recommended for new code */
            uint8_t major;
            uint8_t minor;
            uint8_t patch;
            uint8_t revision;
            uint8_t build;
        };
        struct {
            /* OLD naming -- backward compatible, old callers unchanged */
            uint8_t major1;
            uint8_t major2;
            uint8_t minor_old;          /* was 'minor' in old struct fw_version */
            uint8_t revision_patch_1;
            uint8_t revision_patch_2;
        };
        uint8_t version[5];             /* raw array access */
    };
} __attribute__((packed));

_Static_assert(sizeof(struct eh_fw_version) == 15,
               "eh_fw_version must be exactly 15 bytes");

/* -- Aliases for backward compatibility ----------------------------------- */

/* New preferred typedef */
typedef struct eh_fw_version eh_fw_version_t;

/* Old code uses 'struct fw_version' or 'fw_version_t' -- keep both working */
struct fw_version {
    char project_name[10];
    uint8_t major1;
    uint8_t major2;
    uint8_t minor;
    uint8_t revision_patch_1;
    uint8_t revision_patch_2;
} __attribute__((packed));

typedef struct eh_fw_version fw_version_t;

#endif /* __EH_COMMON_FW_VERSION__H */
