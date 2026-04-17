// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

/*
 * eh_caps.h — Transport-level capability definitions.
 *
 * Canonical cap/ext_cap bit values are #define in eh_common_caps.h.
 * This file adds transport-specific definitions (TLV tags, raw TP, legacy FG).
 */

#ifndef EH_CAPS_H
#define EH_CAPS_H

#include "eh_common_caps.h"

/* ── Raw throughput test flags ─────────────────────────────────────────── */
#define ESP_TEST_RAW_TP_NONE              0
#define ESP_TEST_RAW_TP                   (1 << 0)
#define ESP_TEST_RAW_TP__ESP_TO_HOST      (1 << 1)
#define ESP_TEST_RAW_TP__HOST_TO_ESP      (1 << 2)
#define ESP_TEST_RAW_TP__BIDIRECTIONAL    (1 << 3)

/* ── MCU host TLV tag values ──────────────────────────────────────────── */
#define EH_PRIV_CAPABILITY                0x11
#define EH_PRIV_FIRMWARE_CHIP_ID          0x12
#define EH_PRIV_TEST_RAW_TP              0x13
#define EH_PRIV_RX_Q_SIZE                0x14
#define EH_PRIV_TX_Q_SIZE                0x15
#define EH_PRIV_CAP_EXT                  0x16
#define EH_PRIV_FIRMWARE_VERSION         0x17
#define EH_PRIV_TRANS_SDIO_MODE          0x18
#define EH_PRIV_FEAT_CAPS                0x19

/* ── Linux FG host TLV tag values ─────────────────────────────────────── */
#define ESP_LINUX_FG_PRIV_CAPABILITY      0
#define ESP_LINUX_FG_PRIV_SPI_CLK_MHZ    1
#define ESP_LINUX_FG_PRIV_FIRMWARE_CHIP_ID 2
#define ESP_LINUX_FG_PRIV_TEST_RAW_TP    3
#define ESP_LINUX_FG_PRIV_FW_DATA        4

/* ── Legacy Linux FG boot TLV ─────────────────────────────────────────── */
#include "eh_tlv_defs.h"
#if EH_TLV_V1_LINUX
typedef struct {
    char project_name[3];
    uint8_t major1;
    uint8_t major2;
    uint8_t minor;
    uint8_t revision_patch_1;
    uint8_t revision_patch_2;
} __attribute__((packed)) eh_fw_version_legacy_fg_t;

#define LEGACY_FG_PROJECT_NAME            "FG"
#define LEGACY_FG_PROJECT_VERSION_MAJOR_1 1
#define LEGACY_FG_PROJECT_VERSION_MAJOR_2 0
#define LEGACY_FG_PROJECT_VERSION_MINOR   0
#define LEGACY_FG_PROJECT_REVISION_1      0
#define LEGACY_FG_PROJECT_REVISION_2      0
#endif

/* ── Version macro ────────────────────────────────────────────────────── */
#define EH_VERSION_VAL(major, minor, patch) \
    ((major << 16) | (minor << 8) | (patch))

#endif /* EH_CAPS_H */
