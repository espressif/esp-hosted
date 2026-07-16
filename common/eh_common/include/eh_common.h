// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON__H
#define __ESP_HOSTED_COMMON__H

/* eh_common.h — umbrella for shared wire-contract types. */

#include "eh_common_header.h"
#include "eh_common_header_v2.h"
#include "eh_common_interface.h"
#include "eh_common_caps.h"
#include "eh_tlv_tags.h"
#include "eh_common_fw_version.h"

#define EH_COMMON_MIN(a, b) ((a)<(b)?(a):(b))

/* Project-namespaced bit helpers — IDF's BIT() not always pulled transitively. */
#define EH_SET_BIT(n)              (1U << (n))
#define EH_GET_BIT(v, n)           (((v) >> (n)) & 1U)

#endif /* __ESP_HOSTED_COMMON__H */
