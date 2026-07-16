/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include "esp_netif.h"

extern bool         eh_ex_initialized;
extern esp_netif_t *eh_ex_sta_netif;
extern esp_netif_t *eh_ex_ap_netif;
