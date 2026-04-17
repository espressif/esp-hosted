#include "eh_cp_master_config.h"
#if EH_CP_FEAT_BT_READY
// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "eh_cp_feat_bt_core.h"
#include "eh_cp_feat_bt_uart.h"

#include "esp_log.h"
#if EH_CP_BT_UART
static const char *TAG = "bt_uart";

void eh_cp_feat_bt_init_uart(esp_bt_controller_config_t *cfg)
{
	ESP_LOGI(TAG, "UART%d Pins: TX %d, RX %d, RTS %d, CTS %d Baudrate:%d",
			EH_CP_BT_UART,
			BT_TX_PIN, BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN,
			CONFIG_BTDM_CTRL_HCI_UART_BAUDRATE);

	// init follows examples/bluetooth/hci/controller_hci_uart_esp32/main/controller_hci_uart_demo.c
#if EH_CP_BT_UART == 1
	periph_module_enable(PERIPH_UART1_MODULE);
#elif EH_CP_BT_UART == 2
	periph_module_enable(PERIPH_UART2_MODULE);
#endif

	periph_module_enable(PERIPH_UHCI0_MODULE);

	ESP_ERROR_CHECK(uart_set_pin(EH_CP_BT_UART, BT_TX_PIN,
		BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN));
}
#endif
#endif /* EH_CP_FEAT_BT_READY */
