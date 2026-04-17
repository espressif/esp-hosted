/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_DEFS_H
#define EH_TLV_DEFS_H

/*
 * TLV group macros — derived from Kconfig, used by pack/unpack headers
 * and callers to gate which TLV groups are compiled.
 *
 * Selection is automatic based on RPC version:
 *   RPC V1 Linux  → EH_TLV_V1_LINUX=1
 *   RPC V1 MCU    → EH_TLV_V1_MCU=1
 *   RPC V2        → EH_TLV_V1_MCU=1, EH_TLV_V2=1
 */

#ifdef CONFIG_ESP_HOSTED_TLV_V1_LINUX
#  define EH_TLV_V1_LINUX  1
#else
#  define EH_TLV_V1_LINUX  0
#endif

#ifdef CONFIG_ESP_HOSTED_TLV_V1_MCU
#  define EH_TLV_V1_MCU    1
#else
#  define EH_TLV_V1_MCU    0
#endif

#ifdef CONFIG_ESP_HOSTED_TLV_V2
#  define EH_TLV_V2        1
#else
#  define EH_TLV_V2        0
#endif

#endif /* EH_TLV_DEFS_H */
