// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD

#ifndef EH_CP_FEAT_BT_UART_H
#define EH_CP_FEAT_BT_UART_H

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_BT_READY
#include "esp_bt.h"

void eh_cp_feat_bt_init_uart(esp_bt_controller_config_t *cfg);
#endif

#endif /* EH_CP_FEAT_BT_UART_H */
