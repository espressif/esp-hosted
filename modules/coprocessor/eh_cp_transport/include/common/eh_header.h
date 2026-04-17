/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_header.h — REDIRECT SHIM
 *
 * The canonical definition of struct esp_payload_header and the
 * payload header flag macros lives in:
 *   components/common/eh_common/include/eh_common_header.h
 *
 * This file is a backward-compat shim so that legacy #include paths
 * (e.g. transport sources that include "eh_header.h" directly)
 * continue to compile without modification.
 *
 * DO NOT add new definitions here.
 */

#ifndef __ESP_HOSTED_HEADER__H
#define __ESP_HOSTED_HEADER__H

#include "eh_common_header.h"

#endif /* __ESP_HOSTED_HEADER__H */
