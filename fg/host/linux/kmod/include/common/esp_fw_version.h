// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_FW_VERSION__H
#define __ESP_FW_VERSION__H

#define PROJECT_NAME              "EHCP"
#define PROJECT_VERSION_MAJOR_1   3
#define PROJECT_VERSION_MAJOR_2   0
#define PROJECT_VERSION_MINOR     0
#define PROJECT_REVISION_PATCH_1  0
#define PROJECT_REVISION_PATCH_2  0

struct fw_version {
    char    project_name[10];
    uint8_t major1;
    uint8_t major2;
    uint8_t minor;
    uint8_t revision_patch_1;
    uint8_t revision_patch_2;
};

#endif
