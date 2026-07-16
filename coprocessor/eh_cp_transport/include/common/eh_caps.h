/* SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 *
 * Transport-level cap bit flags. TLV tag IDs (EH_PRIV_*, ESP_PRIV_*,
 * ESP_LINUX_802_3_PRIV_*) and the legacy FG fw-version struct live in
 * common/eh_tlv/include/eh_tlv_tags.h — single owner.
 * Cap bit defs (ESP_*_SUPPORT etc.) live in eh_common_caps.h.
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

/* ── Version macro ────────────────────────────────────────────────────── */
#define EH_VERSION_VAL(major, minor, patch) \
    ((major << 16) | (minor << 8) | (patch))

#endif /* EH_CAPS_H */
