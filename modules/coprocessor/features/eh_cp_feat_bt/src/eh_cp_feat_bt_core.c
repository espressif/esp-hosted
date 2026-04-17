/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_BT_READY
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bt.h"
#endif
#include "eh_cp_feat_bt_core.h"
#include "eh_cp_feat_bt_uart.h"
#include "eh_transport_cp.h"
#include "eh_transport.h"
#include "eh_caps.h"
#include "eh_interface.h"

#if EH_CP_FEAT_BT_READY
#include "esp_log.h"
#include "eh_log.h"
#include "soc/lldesc.h"
#include "esp_mac.h"

static const char *TAG = "h_bt";

#if EH_CP_BT_HCI
/* ***** HCI specific part ***** */

#define VHCI_MAX_TIMEOUT_MS 	2000
static SemaphoreHandle_t vhci_send_sem;

static void controller_rcv_pkt_ready(void)
{
	if (vhci_send_sem)
		xSemaphoreGive(vhci_send_sem);
}

static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
	interface_buffer_handle_t buf_handle;
	uint8_t *buf = NULL;

	buf = (uint8_t *) malloc(len);

	if (!buf) {
		ESP_LOGE(TAG, "HCI Send packet: memory allocation failed");
		return ESP_FAIL;
	}

	memcpy(buf, data, len);

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = ESP_HCI_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buf;
	buf_handle.priv_buffer_handle = buf;
	buf_handle.free_buf_handle = free;

	ESP_HEXLOGV("bt_tx new", data, len, 32);

	if (send_to_host_queue(&buf_handle, PRIO_Q_BT)) {
		free(buf);
		return ESP_FAIL;
	}

	return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
	.notify_host_send_available = controller_rcv_pkt_ready,
	.notify_host_recv = host_rcv_pkt
};

void process_hci_rx_pkt(uint8_t *payload, uint16_t payload_len)
{
	/* VHCI needs one extra byte at the start of payload */
	/* that is accommodated in esp_payload_header */
	ESP_HEXLOGV("bt_rx", payload, payload_len, 32);

	payload--;
	payload_len++;

	if (!esp_vhci_host_check_send_available()) {
		ESP_LOGD(TAG, "VHCI not available");
	}

#if SOC_ESP_NIMBLE_CONTROLLER
	esp_vhci_host_send_packet(payload, payload_len);
#else
	if (vhci_send_sem) {
		if (xSemaphoreTake(vhci_send_sem, VHCI_MAX_TIMEOUT_MS) == pdTRUE) {
			esp_vhci_host_send_packet(payload, payload_len);
		} else {
			ESP_LOGI(TAG, "VHCI sem timeout");
		}
	}
#endif
}

#elif EH_CP_BT_UART
/* ***** UART specific part ***** */

#endif

#if EH_CP_BT_HCI
#if SOC_ESP_NIMBLE_CONTROLLER

#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 3, 0)
#include "ble_hci_trans.h"

typedef enum {
	DATA_TYPE_COMMAND = 1,
	DATA_TYPE_ACL     = 2,
	DATA_TYPE_SCO     = 3,
	DATA_TYPE_EVENT   = 4
} serial_data_type_t;

/* Host-to-controller command. */
#define BLE_HCI_TRANS_BUF_CMD       3

/* ACL_DATA_MBUF_LEADINGSPACE: The leadingspace in user info header for ACL data */
#define ACL_DATA_MBUF_LEADINGSPACE    4

void esp_vhci_host_send_packet(uint8_t *data, uint16_t len)
{
	if (*(data) == DATA_TYPE_COMMAND) {
		struct ble_hci_cmd *cmd = NULL;
		cmd = (struct ble_hci_cmd *) ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_CMD);
		if (!cmd) {
			ESP_LOGE(TAG, "Failed to allocate memory for HCI transport buffer");
			return;
		}

		memcpy((uint8_t *)cmd, data + 1, len - 1);
		ble_hci_trans_hs_cmd_tx((uint8_t *)cmd);
	}

	if (*(data) == DATA_TYPE_ACL) {
		struct os_mbuf *om = os_msys_get_pkthdr(len, ACL_DATA_MBUF_LEADINGSPACE);
		assert(om);
		os_mbuf_append(om, &data[1], len - 1);
		ble_hci_trans_hs_acl_tx(om);
	}

}

bool esp_vhci_host_check_send_available() {
	// not need to check in esp new controller
	return true;
}

int
ble_hs_hci_rx_evt(uint8_t *hci_ev, void *arg)
{
	uint16_t len = hci_ev[1] + 3;
	uint8_t *data = (uint8_t *)malloc(len);
	data[0] = 0x04;
	memcpy(&data[1], hci_ev, len - 1);
	ble_hci_trans_buf_free(hci_ev);
	vhci_host_cb.notify_host_recv(data, len);
	free(data);
	return 0;
}


int
ble_hs_rx_data(struct os_mbuf *om, void *arg)
{
	uint16_t len = om->om_len + 1;
	uint8_t *data = (uint8_t *)malloc(len);
	data[0] = 0x02;
	os_mbuf_copydata(om, 0, len - 1, &data[1]);
	vhci_host_cb.notify_host_recv(data, len);
	free(data);
	os_mbuf_free_chain(om);
	return 0;
}
#endif // ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 3, 0)

#endif /* SOC_ESP_NIMBLE_CONTROLLER */
#endif /* EH_CP_BT_HCI */
#endif

esp_err_t eh_cp_bt_init(void)
{
	esp_err_t ret = ESP_FAIL;

#if EH_CP_FEAT_BT_READY
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BT));
	ESP_LOGI(TAG, "ESP Bluetooth MAC addr: %02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

#ifdef EH_CP_BT_UART
	eh_cp_feat_bt_init_uart(&bt_cfg);
#endif

	ret = esp_bt_controller_init(&bt_cfg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_bt_controller_init() FAILED");
		return ret;
	}

#if EH_CP_BT_HCI
	vhci_send_sem = xSemaphoreCreateBinary();
	if (vhci_send_sem == NULL) {
		ESP_LOGE(TAG, "Failed to create VHCI send sem");
		return ESP_ERR_NO_MEM;
	}

	xSemaphoreGive(vhci_send_sem);
#endif
#endif

	return ret;
}

esp_err_t eh_cp_bt_deinit(bool mem_release)
{
	esp_err_t result = ESP_FAIL;

#if EH_CP_FEAT_BT_READY
#if EH_CP_BT_HCI
	if (vhci_send_sem) {
		/* Dummy take and give sema before deleting it */
		xSemaphoreTake(vhci_send_sem, portMAX_DELAY);
		xSemaphoreGive(vhci_send_sem);
		vSemaphoreDelete(vhci_send_sem);
		vhci_send_sem = NULL;
	}
#endif
	result = esp_bt_controller_deinit();
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "esp_bt_controller_deinit FAILED");
		return result;
	}

	if (mem_release) {
		result = ESP_OK;
#if EH_CP_BT_MODE_BLE
		result = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
#elif EH_CP_BT_MODE_BT
		result = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#elif EH_CP_BT_MODE_DUAL
		result = esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
#endif
		if (result != ESP_OK) {
			ESP_LOGE(TAG, "esp_bt_controller_mem_release FAILED");
			return result;
		}
	}
#endif
	return result;
}

esp_err_t eh_cp_bt_enable(void)
{
	esp_err_t ret = ESP_FAIL;

#if EH_CP_FEAT_BT_READY
#if EH_CP_BT_MODE_BLE
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
#elif EH_CP_BT_MODE_BT
	ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
#elif EH_CP_BT_MODE_DUAL
	ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
#endif
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_bt_controller_enable FAILED (or not called)");
		return ret;
	}

#if EH_CP_BT_HCI
#if SOC_ESP_NIMBLE_CONTROLLER && (ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 3, 0))
    ble_hci_trans_cfg_hs((ble_hci_trans_rx_cmd_fn *)ble_hs_hci_rx_evt,NULL,
                         (ble_hci_trans_rx_acl_fn *)ble_hs_rx_data,NULL);
#else
	ret = esp_vhci_host_register_callback(&vhci_host_cb);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register VHCI callback");
		return ret;
	}
#endif
#endif // EH_CP_BT_HCI
#endif // EH_CP_FEAT_BT_READY

	return ret;
}

esp_err_t eh_cp_bt_disable(void)
{
#if EH_CP_FEAT_BT_READY
#if EH_CP_BT_HCI
// unregister callback functions
#if SOC_ESP_NIMBLE_CONTROLLER && (ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(5, 3, 0))
    ble_hci_trans_cfg_hs(NULL, NULL,
                         NULL, NULL);
#else
	esp_vhci_host_callback_t null_cb = {
		NULL, NULL };
	esp_vhci_host_register_callback(&null_cb);
#endif
#endif

	return esp_bt_controller_disable();
#else
	return ESP_FAIL;
#endif
}

uint8_t eh_cp_bt_get_capabilities(void)
{
	uint8_t cap = 0;
#if EH_CP_FEAT_BT_READY
	ESP_LOGI(TAG, "- BT/BLE");
#if EH_CP_BT_HCI

#if EH_CP_TRANSPORT_SPI
	ESP_LOGI(TAG, "   - HCI Over SPI");
	cap |= ESP_BT_SPI_SUPPORT;
#elif EH_CP_TRANSPORT_SDIO
	ESP_LOGI(TAG, "   - HCI Over SDIO");
	cap |= ESP_BT_SDIO_SUPPORT;
#endif /* EH_CP_TRANSPORT_SDIO */

#elif EH_CP_BT_UART
	ESP_LOGI(TAG, "   - HCI Over UART");
	cap |= ESP_BT_UART_SUPPORT;
#endif /* EH_CP_BT_UART */

#if EH_CP_BT_MODE_BLE
	ESP_LOGI(TAG, "   - BLE only");
	cap |= ESP_BLE_ONLY_SUPPORT;
#elif EH_CP_BT_MODE_BT
	ESP_LOGI(TAG, "   - BR_EDR only");
	cap |= ESP_BR_EDR_ONLY_SUPPORT;
#elif EH_CP_BT_MODE_DUAL
	ESP_LOGI(TAG, "   - BT/BLE dual mode");
	cap |= ESP_BLE_ONLY_SUPPORT | ESP_BR_EDR_ONLY_SUPPORT;
#endif /* EH_CP_BT_MODE_DUAL */
#endif

	return cap;
}

uint32_t eh_cp_bt_get_ext_capabilities(void)
{
	uint32_t ext_cap = 0;
#if EH_CP_FEAT_BT_READY
	ESP_LOGI(TAG, "- BT/BLE (extended)");
#if EH_CP_BT_HCI
#if EH_CP_TRANSPORT_SPI_HD
	ESP_LOGI(TAG, "   - HCI Over SPI HD");
	ext_cap |= EH_EXT_CAP_BT_INTERFACE;
#endif
#if EH_CP_TRANSPORT_UART
	ESP_LOGI(TAG, "   - HCI Over UART (VHCI)");
	ext_cap |= EH_EXT_CAP_BT_INTERFACE;
#endif
#endif
#endif
	return ext_cap;
}
