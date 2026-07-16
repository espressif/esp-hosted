/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_DEFS_H
#define EH_TLV_DEFS_H

/* TLV group flags from Kconfig — gate pack/unpack inclusion per RPC version. */

#ifdef CONFIG_ESP_HOSTED_TLV_V1
#  define EH_TLV_V1  1
#else
#  define EH_TLV_V1  0
#endif

#ifdef CONFIG_ESP_HOSTED_TLV_V2
#  define EH_TLV_V2    1
#else
#  define EH_TLV_V2    0
#endif

#ifdef CONFIG_ESP_HOSTED_TLV_V3
#  define EH_TLV_V3        1
#else
#  define EH_TLV_V3        0
#endif

#endif /* EH_TLV_DEFS_H */
