/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_feat_bt.h"
#include "eh_cp_core.h"          /* EH_CP_FEAT_REGISTER */
#include "eh_common_caps.h"
#include "eh_cp_master_config.h"
#include "eh_cp_feat_bt_core.h"
#include "esp_log.h"

static const char *TAG = "ext_feat_bt";

#if EH_CP_FEAT_BT_READY
static esp_err_t bt_hci_rx_cb(void *ctx, void *buf, uint16_t len, void *eb)
{
	(void)ctx;
	(void)eb;
#if EH_CP_BT_HCI
	process_hci_rx_pkt((uint8_t *)buf, len);
#else
	(void)buf;
	(void)len;
#endif
	return ESP_OK;
}
#endif

static esp_err_t eh_cp_feat_bt_common_init(void)
{
#if EH_CP_FEAT_BT_READY
	eh_cp_add_feat_cap_bits(eh_cp_bt_get_capabilities(),
	                           eh_cp_bt_get_ext_capabilities());
	eh_cp_add_feat_cap_bits_idx(EH_FEAT_IDX_BT, 0x1u /* BT_BASIC */);
#if EH_CP_BT_HCI
	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_register_rx_cb(ESP_HCI_IF, bt_hci_rx_cb, NULL));
#endif
#if EH_CP_AUTO_START_BT
	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_bt_init());
	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_bt_enable());
#endif
	ESP_LOGI(TAG, "BT feature extension init ok");
	return ESP_OK;
#else
	ESP_LOGW(TAG, "BT feature extension init skipped (BT not enabled)");
	return ESP_OK;
#endif
}

static esp_err_t eh_cp_feat_bt_common_deinit(void)
{
#if EH_CP_FEAT_BT_READY
	eh_cp_clear_feat_cap_bits_idx(EH_FEAT_IDX_BT, 0x1u /* BT_BASIC */);
	eh_cp_clear_feat_cap_bits(eh_cp_bt_get_capabilities(),
	                             eh_cp_bt_get_ext_capabilities());
#if EH_CP_AUTO_STOP_BT
	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_bt_disable());
	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_bt_deinit(true));
#endif
	ESP_LOGI(TAG, "BT feature extension deinit ok");
	return ESP_OK;
#else
	ESP_LOGW(TAG, "BT feature extension deinit skipped (BT not enabled)");
	return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_bt_init(void)
{
	return eh_cp_feat_bt_common_init();
}

esp_err_t eh_cp_feat_bt_deinit(void)
{
	return eh_cp_feat_bt_common_deinit();
}

#if EH_CP_FEAT_BT_HCI_VHCI_READY
static esp_err_t eh_cp_feat_bt_hosted_hci_init(void)
{
	return eh_cp_feat_bt_common_init();
}

static esp_err_t eh_cp_feat_bt_hosted_hci_deinit(void)
{
	return eh_cp_feat_bt_common_deinit();
}
#endif

#if EH_CP_FEAT_BT_HCI_UART_READY
static esp_err_t eh_cp_feat_bt_standard_hci_init(void)
{
	return eh_cp_feat_bt_common_init();
}

static esp_err_t eh_cp_feat_bt_standard_hci_deinit(void)
{
	return eh_cp_feat_bt_common_deinit();
}
#endif

#if EH_CP_FEAT_BT_AUTO_INIT && EH_CP_FEAT_BT_HCI_VHCI_READY
EH_CP_FEAT_REGISTER(eh_cp_feat_bt_hosted_hci_init,
                   eh_cp_feat_bt_hosted_hci_deinit,
                   "bt_hosted_hci", tskNO_AFFINITY, 200);
#endif

#if EH_CP_FEAT_BT_AUTO_INIT && EH_CP_FEAT_BT_HCI_UART_READY
EH_CP_FEAT_REGISTER(eh_cp_feat_bt_standard_hci_init,
                   eh_cp_feat_bt_standard_hci_deinit,
                   "bt_std_hci", tskNO_AFFINITY, 200);
#endif
