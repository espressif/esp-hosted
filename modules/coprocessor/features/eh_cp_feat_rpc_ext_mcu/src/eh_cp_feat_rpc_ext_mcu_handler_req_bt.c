/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "eh_transport.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_SOC_BT_SUPPORTED)
#include "esp_bt.h"
#endif

//static const char* TAG = "mcu_bt_rpc";

/* 
 * ========================================
 * COPY BLUETOOTH RPC HANDLERS HERE
 * ========================================
 * 
 * Functions to copy (if they exist in slave_control.c):
 * - req_bt_init()
 * - req_bt_deinit() 
 * - req_bt_enable()
 * - req_bt_disable()
 * - All other req_bt_* functions
 * 
 */
#endif /* EH_CP_FEAT_RPC_MCU_READY */
