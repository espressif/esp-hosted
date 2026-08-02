/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* BT host feature helpers (all host types). Controller lifecycle RPC lives in
 * eh_host_bt.c; the HCI byte pipe is MCU-only, in eh_host_feat_bt_mcu.c. There
 * is no BT auto-init: the esp_hosted_bt adapter drives setup explicitly and
 * calls eh_host_bt_apply_mac() before bringing the controller up. */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "eh_host_feat_bt.h"
#include "eh_host_sys.h"      /* eh_host_iface_mac_addr_set */

static const char *TAG = "eh_host_feat_bt";

static int parse_mac_str(const char *s, uint8_t out[6])
{
    if (!s || !s[0]) return -1;
    unsigned v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
        return -1;
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xff) return -1;
        out[i] = (uint8_t)v[i];
    }
    return 0;
}

esp_err_t eh_host_bt_apply_mac(const uint8_t *mac6)
{
    uint8_t kmac[6];
    const uint8_t *use = mac6;                 /* runtime value wins */
    if (!use && parse_mac_str(CONFIG_ESP_HOSTED_HOST_FEAT_BT_MAC, kmac) == 0)
        use = kmac;                            /* else the Kconfig default */
    if (!use)
        return ESP_OK;                         /* else leave as-is */

    esp_err_t e = eh_host_iface_mac_addr_set((uint8_t *)(uintptr_t)use, 6, ESP_MAC_BT);
    if (e != ESP_OK)
        ESP_LOGW(TAG, "set BT MAC failed: 0x%x", e);
    return e;
}
