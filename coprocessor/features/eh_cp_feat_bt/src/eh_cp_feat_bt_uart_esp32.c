// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_BT_READY

#include "eh_cp_feat_bt_core.h"
#include "eh_cp_feat_bt_uart.h"

#include "esp_log.h"
#include "eh_check.h"
#if EH_CP_BT_UART
static const char *TAG = "bt_uart";

void eh_cp_feat_bt_init_uart(esp_bt_controller_config_t *cfg)
{
	ESP_LOGI(TAG, "UART%d Pins: TX %d, RX %d, RTS %d, CTS %d Baudrate:%d",
			EH_CP_BT_UART,
			BT_TX_PIN, BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN,
			CONFIG_BTDM_CTRL_HCI_UART_BAUDRATE);

#if EH_CP_BT_UART == 1
	periph_module_enable(PERIPH_UART1_MODULE);
#elif EH_CP_BT_UART == 2
	periph_module_enable(PERIPH_UART2_MODULE);
#endif

	periph_module_enable(PERIPH_UHCI0_MODULE);

	EH_CHECK_OK(uart_set_pin(EH_CP_BT_UART, BT_TX_PIN,
		BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN));
}
#endif
#endif /* EH_CP_FEAT_BT_READY */
