// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_BT_READY

#include "eh_cp_feat_bt_core.h"
#include "eh_cp_feat_bt_uart.h"

#include "esp_log.h"

#if EH_CP_BT_UART
static const char *TAG = "bt_uart";

void eh_cp_feat_bt_init_uart(esp_bt_controller_config_t *cfg)
{
	ESP_LOGI(TAG, "UART%d Pins: Tx:%d Rx:%d", EH_CP_BT_UART,
			BT_TX_PIN, BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN);

	/* Specific uart init handled by BT HCI UART controller. */
}
#endif
#endif /* EH_CP_FEAT_BT_READY */
