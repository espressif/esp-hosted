/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_interface.h — REDIRECT SHIM
 *
 * The canonical definition of eh_if_type_t lives in:
 *   components/common/eh_common/include/eh_common_interface.h
 *
 * This file is a backward-compat shim so that legacy #include paths
 * (e.g. transport sources that include "eh_interface.h" directly)
 * continue to compile without modification.
 *
 * DO NOT add new definitions here.
 */

#ifndef __ESP_HOSTED_INTERFACE_H__
#define __ESP_HOSTED_INTERFACE_H__

#include "eh_common_interface.h"

#endif /* __ESP_HOSTED_INTERFACE_H__ */
