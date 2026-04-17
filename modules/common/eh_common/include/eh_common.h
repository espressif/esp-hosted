// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON__H
#define __ESP_HOSTED_COMMON__H

/*
 * eh_common.h — umbrella header for all common wire-contract types
 *
 * Include this single header to get all shared definitions:
 *   - V1 wire header struct (esp_payload_header)
 *   - V2 wire header struct (eh_header_v2_t)
 *   - Interface type enum   (eh_if_type_t)
 *   - Capability bitmasks   (caps / ext_caps / feat_caps)
 *   - PRIV TLV type codes   (ESP_PRIV_*)
 *   - Firmware version struct
 *
 * Components that only need one subset may include the individual headers
 * (eh_common_header.h, eh_common_caps.h, etc.) directly.
 */

#include "eh_common_header.h"
#include "eh_common_header_v2.h"
#include "eh_common_interface.h"
#include "eh_common_caps.h"
#include "eh_common_tlv.h"
#include "eh_common_fw_version.h"

#define EH_COMMON_MIN(a, b) ((a)<(b)?(a):(b))

#endif /* __ESP_HOSTED_COMMON__H */
