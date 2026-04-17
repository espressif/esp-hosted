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

#include <string.h>

#include "soc/lldesc.h"
// #include "esp_private/periph_ctrl.h"
#include "esp_private/gdma.h"
#include "hal/uhci_ll.h"
#include "driver/gpio.h"

#include "esp_log.h"
#if EH_CP_BT_UART
static const char *TAG = "bt_uart";

// Operation functions for HCI UART Transport Layer
static bool hci_uart_tl_init(void);
static void hci_uart_tl_deinit(void);
static void hci_uart_tl_recv_async(uint8_t *buf, uint32_t size, esp_bt_hci_tl_callback_t callback, void *arg);
static void hci_uart_tl_send_async(uint8_t *buf, uint32_t size, esp_bt_hci_tl_callback_t callback, void *arg);
static void hci_uart_tl_flow_on(void);
static bool hci_uart_tl_flow_off(void);
static void hci_uart_tl_finish_transfers(void);

struct uart_txrxchannel {
	esp_bt_hci_tl_callback_t callback;
	void *arg;
	lldesc_t link;
};

struct uart_env_tag {
	struct uart_txrxchannel tx;
	struct uart_txrxchannel rx;
};

static struct uart_env_tag uart_env;

static volatile uhci_dev_t *s_uhci_hw = &UHCI0;
static gdma_channel_handle_t s_rx_channel;
static gdma_channel_handle_t s_tx_channel;

static esp_bt_hci_tl_t s_hci_uart_tl_funcs = {
	._magic = ESP_BT_HCI_TL_MAGIC_VALUE,
	._version = ESP_BT_HCI_TL_VERSION,
	._reserved = 0,
	._open = (void *)hci_uart_tl_init,
	._close = (void *)hci_uart_tl_deinit,
	._finish_transfers = (void *)hci_uart_tl_finish_transfers,
	._recv = (void *)hci_uart_tl_recv_async,
	._send = (void *)hci_uart_tl_send_async,
	._flow_on = (void *)hci_uart_tl_flow_on,
	._flow_off = (void *)hci_uart_tl_flow_off,
};

static bool hci_uart_tl_init(void)
{
	return true;
}

static void hci_uart_tl_deinit(void)
{
}

static IRAM_ATTR void hci_uart_tl_recv_async(uint8_t *buf, uint32_t size,
		esp_bt_hci_tl_callback_t callback, void *arg)
{
	assert(buf != NULL);
	assert(size != 0);
	assert(callback != NULL);
	uart_env.rx.callback = callback;
	uart_env.rx.arg = arg;

	memset(&uart_env.rx.link, 0, sizeof(lldesc_t));
	uart_env.rx.link.buf = buf;
	uart_env.rx.link.size = size;

	s_uhci_hw->pkt_thres.thrs = size;

	gdma_start(s_rx_channel, (intptr_t)(&uart_env.rx.link));
}

static IRAM_ATTR void hci_uart_tl_send_async(uint8_t *buf, uint32_t size,
		esp_bt_hci_tl_callback_t callback, void *arg)
{
	assert(buf != NULL);
	assert(size != 0);
	assert(callback != NULL);

	uart_env.tx.callback = callback;
	uart_env.tx.arg = arg;

	memset(&uart_env.tx.link, 0, sizeof(lldesc_t));
	uart_env.tx.link.length = size;
	uart_env.tx.link.buf = buf;
	uart_env.tx.link.eof = 1;

	gdma_start(s_tx_channel, (intptr_t)(&uart_env.tx.link));
}

static void hci_uart_tl_flow_on(void)
{
}

static bool hci_uart_tl_flow_off(void)
{
	return true;
}

static void hci_uart_tl_finish_transfers(void)
{
}

static IRAM_ATTR bool hci_uart_tl_rx_eof_callback(gdma_channel_handle_t dma_chan,
		gdma_event_data_t *event_data, void *user_data)
{
	assert(dma_chan == s_rx_channel);
	assert(uart_env.rx.callback != NULL);
	esp_bt_hci_tl_callback_t callback = uart_env.rx.callback;
	void *arg = uart_env.rx.arg;

	// clear callback pointer
	uart_env.rx.callback = NULL;
	uart_env.rx.arg = NULL;

	// call handler
	callback(arg, ESP_BT_HCI_TL_STATUS_OK);

	// send notification to Bluetooth Controller task
	esp_bt_h4tl_eif_io_event_notify(1);

	return true;
}

static IRAM_ATTR bool hci_uart_tl_tx_eof_callback(gdma_channel_handle_t dma_chan,
		gdma_event_data_t *event_data, void *user_data)
{
	assert(dma_chan == s_tx_channel);
	assert(uart_env.tx.callback != NULL);
	esp_bt_hci_tl_callback_t callback = uart_env.tx.callback;
	void *arg = uart_env.tx.arg;

	// clear callback pointer
	uart_env.tx.callback = NULL;
	uart_env.tx.arg = NULL;

	// call handler
	callback(arg, ESP_BT_HCI_TL_STATUS_OK);

	// send notification to Bluetooth Controller task
	esp_bt_h4tl_eif_io_event_notify(1);

	return true;
}

void eh_cp_feat_bt_init_uart(esp_bt_controller_config_t *cfg)
{
	ESP_LOGD(TAG, "Set-up BLE for ESP32-C3/ESP32-S3");

	cfg->hci_tl_funcs = &s_hci_uart_tl_funcs;

#if EH_CP_BT_UART == 1
	periph_module_enable(PERIPH_UART1_MODULE);
	periph_module_reset(PERIPH_UART1_MODULE);
#elif EH_CP_BT_UART == 2
	periph_module_enable(PERIPH_UART2_MODULE);
	periph_module_reset(PERIPH_UART2_MODULE);
#else
#error "Invalid UART Port Selected"
#endif

	periph_module_enable(PERIPH_UHCI0_MODULE);
	periph_module_reset(PERIPH_UHCI0_MODULE);

	gpio_config_t io_output_conf = {
		.intr_type = DISABLE_INTR_ON_GPIO,    /* Disable interrupt */
		.mode = GPIO_MODE_OUTPUT,             /* Output mode */
		.pin_bit_mask = GPIO_OUTPUT_PIN_SEL,  /* Bit mask of the output pins */
		.pull_down_en = 0,                    /* Disable pull-down mode */
		.pull_up_en = 0,                      /* Disable pull-up mode */
	};
	gpio_config(&io_output_conf);

	gpio_config_t io_input_conf = {
		.intr_type = DISABLE_INTR_ON_GPIO,    /* Disable interrupt */
		.mode = GPIO_MODE_INPUT,              /* Input mode */
		.pin_bit_mask = GPIO_INPUT_PIN_SEL,   /* Bit mask of the input pins */
		.pull_down_en = 0,                    /* Disable pull-down mode */
		.pull_up_en = 0,                      /* Disable pull-down mode */
	};
	gpio_config(&io_input_conf);

	ESP_ERROR_CHECK( uart_set_pin(EH_CP_BT_UART, BT_TX_PIN,
			BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN) );
	ESP_LOGI(TAG, "UART%d Pins: Tx:%d Rx:%d RTS:%d CTS:%d",
			EH_CP_BT_UART, BT_TX_PIN, BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN);

	// configure UART
	ESP_LOGI(TAG, "baud rate for HCI uart :: %d \n", BT_BAUDRATE);

	uart_config_t uart_config = {
		.baud_rate = BT_BAUDRATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = BT_FLOWCTRL,
		.rx_flow_ctrl_thresh = UART_RX_THRS,
		.source_clk = UART_SCLK_APB,
	};
	ESP_ERROR_CHECK(uart_param_config(EH_CP_BT_UART, &uart_config));

	// install DMA driver
	gdma_channel_alloc_config_t tx_channel_config = {
		.flags.reserve_sibling = 1,
		.direction = GDMA_CHANNEL_DIRECTION_TX,
	};
	ESP_ERROR_CHECK(gdma_new_channel(&tx_channel_config, &s_tx_channel));

	gdma_channel_alloc_config_t rx_channel_config = {
		.direction = GDMA_CHANNEL_DIRECTION_RX,
		.sibling_chan = s_tx_channel,
	};
	ESP_ERROR_CHECK(gdma_new_channel(&rx_channel_config, &s_rx_channel));

	gdma_connect(s_tx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_UHCI, 0));
	gdma_connect(s_rx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_UHCI, 0));

	gdma_strategy_config_t strategy_config = {
		.auto_update_desc = false,
		.owner_check = false
	};
	gdma_apply_strategy(s_tx_channel, &strategy_config);
	gdma_apply_strategy(s_rx_channel, &strategy_config);

	gdma_rx_event_callbacks_t rx_cbs = {
		.on_recv_eof = hci_uart_tl_rx_eof_callback
	};
	gdma_register_rx_event_callbacks(s_rx_channel, &rx_cbs, NULL);

	gdma_tx_event_callbacks_t tx_cbs = {
		.on_trans_eof = hci_uart_tl_tx_eof_callback
	};
	gdma_register_tx_event_callbacks(s_tx_channel, &tx_cbs, NULL);

	// configure UHCI
	uhci_ll_init(s_uhci_hw);
	uhci_ll_set_eof_mode(s_uhci_hw, UHCI_RX_LEN_EOF);
	// disable software flow control
	s_uhci_hw->escape_conf.val = 0;
	uhci_ll_attach_uart_port(s_uhci_hw, 1);
}
#endif // EH_CP_BT_UART
#endif /* EH_CP_FEAT_BT_READY */
