/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_EXT_V2_READY
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "eh_transport.h"
#include "eh_cp_feat_rpc_ext_v2.h"
#include "eh_cp_feat_rpc_ext_v2_priv.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_SOC_BT_SUPPORTED)
#include "esp_bt.h"
#endif

/* TODO: BT RPC handlers (req_bt_init/deinit/enable/disable/...). */
#endif /* EH_CP_FEAT_RPC_EXT_V2_READY */
