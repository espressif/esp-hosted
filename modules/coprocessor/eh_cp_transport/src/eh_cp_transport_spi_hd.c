// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "eh_cp_master_config.h"

#include "eh_tlv.h"
#include "eh_tlv_defs.h"
#include "eh_tlv_v1_linux.h"
#include "eh_tlv_v1_mcu.h"
#include "eh_tlv_v2.h"

#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/spi_slave_hd.h"

#include "eh_transport_cp.h"
#include "endian.h"
#include "eh_mempool.h"
#include "eh_interface.h"
#include "eh_header.h"
#include "eh_transport.h"
#include "eh_caps.h"
#include "eh_common_tlv.h"         /* ESP_PRIV_HEADER_VERSION TLV codes */
#include "eh_frame.h"       /* eh_frame_encode/decode/init */
#include "eh_transport_spi_hd.h"
//#include "eh_cp_fw_ver.h"
#include "eh_common_fw_version.h"
#include "eh_transport_cp.h"
#include "eh_cp_transport_utils.h"

#include "esp_log.h"
static const char TAG[] = "SPI_HD_DRIVER";

/* SPI HD settings */
#define NUM_DATA_BITS              CONFIG_EH_TRANSPORT_CP_SPI_HD_NUM_DATA_LINES

#define ESP_SPI_HD_MODE            CONFIG_EH_TRANSPORT_CP_SPI_HD_MODE_VALUE
#define GPIO_CS                    CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_CS
#define GPIO_SCLK                  CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_CLK
#define GPIO_D0                    CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D0
#define GPIO_D1                    CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D1
#if (NUM_DATA_BITS == 4)
#define GPIO_D2                    CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D2
#define GPIO_D3                    CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D3
#endif
#define GPIO_DATA_READY            CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_DATA_READY

#define TX_MEMPOOL_NUM_BLOCKS      CONFIG_EH_TRANSPORT_CP_SPI_HD_Q_SIZE
#define RX_MEMPOOL_NUM_BLOCKS      CONFIG_EH_TRANSPORT_CP_SPI_HD_Q_SIZE

#define SPI_HD_CHECKSUM            CONFIG_EH_TRANSPORT_CP_SPI_HD_CHECKSUM

#define SPI_HOST                   SPI2_HOST // only SPI2 can be used in SPI HD

/** ESP SPI Half-duplex Protocol:
 * 8 command bits
 * 8 address bits
 * 8 dummy bits
 * x data bits (in 32-bit words)
 */
#define NUM_COMMAND_BITS           8
#define NUM_ADDRESS_BITS           8
#define NUM_DUMMY_BITS             8

/* By default Data Ready use Active High,
 * unless configured otherwise.
 * For Active low, set value as 0 */
#define H_DATAREADY_ACTIVE_HIGH    CONFIG_ESP_DR_ACTIVE_HIGH

/* SPI-DMA settings */
#define SPI_HD_DMA_ALIGNMENT_BYTES   4
#define SPI_HD_DMA_ALIGNMENT_MASK    (SPI_HD_DMA_ALIGNMENT_BYTES-1)
#define IS_SPI_HD_DMA_ALIGNED(VAL)   (!((VAL)& SPI_HD_DMA_ALIGNMENT_MASK))
#define MAKE_SPI_HD_DMA_ALIGNED(VAL) (VAL += SPI_HD_DMA_ALIGNMENT_BYTES - \
				((VAL)& SPI_HD_DMA_ALIGNMENT_MASK))

#define DMA_CHAN SPI_DMA_CH_AUTO // automatically select the DMA channel

#define SPI_HD_BUFFER_SIZE          MAX_TRANSPORT_BUF_SIZE
#define SPI_HD_QUEUE_SIZE           CONFIG_EH_TRANSPORT_CP_SPI_HD_Q_SIZE

#define GPIO_MASK_DATA_READY        (1ULL << GPIO_DATA_READY)

#if H_DATAREADY_ACTIVE_HIGH
  #define H_DR_VAL_ACTIVE           GPIO_OUT_W1TS_REG
  #define H_DR_VAL_INACTIVE         GPIO_OUT_W1TC_REG
  #define H_DR_PULL_REGISTER        GPIO_PULLDOWN_ONLY
#else
  #define H_DR_VAL_ACTIVE           GPIO_OUT_W1TC_REG
  #define H_DR_VAL_INACTIVE         GPIO_OUT_W1TS_REG
  #define H_DR_PULL_REGISTER        GPIO_PULLUP_ONLY
#endif

#if H_DATAREADY_ACTIVE_HIGH
  #define set_dataready_gpio()     { ESP_EARLY_LOGV(TAG, "set_dataready_gpio"); data_ready_gpio_active = true; gpio_set_level(GPIO_DATA_READY, 1); }
  #define reset_dataready_gpio()   { ESP_EARLY_LOGV(TAG, "reset_dataready_gpio"); gpio_set_level(GPIO_DATA_READY, 0); data_ready_gpio_active = false; }
#else
  #define set_dataready_gpio()     { ESP_EARLY_LOGV(TAG, "set_dataready_gpio"); data_ready_gpio_active = true; gpio_set_level(GPIO_DATA_READY, 0); }
  #define reset_dataready_gpio()   { ESP_EARLY_LOGV(TAG, "reset_dataready_gpio"); gpio_set_level(GPIO_DATA_READY, 1); data_ready_gpio_active = false; }
#endif

// for flow control
static volatile uint8_t wifi_flow_ctrl = 0;
static void flow_ctrl_task(void* pvParameters);
static SemaphoreHandle_t flow_ctrl_sem = NULL;
#define TRIGGER_FLOW_CTRL() if(flow_ctrl_sem) xSemaphoreGive(flow_ctrl_sem);

static volatile bool data_ready_gpio_active = false;
static interface_context_t context;
static interface_handle_t if_handle_g;
static SemaphoreHandle_t spi_hd_rx_sem;
static QueueHandle_t spi_hd_rx_queue[MAX_PRIORITY_QUEUES];

static void spi_hd_rx_task(void* pvParameters);
static void spi_hd_tx_done_task(void* pvParameters);

static uint32_t tx_ready_buf_size = 0;
static uint32_t rx_ready_buf_num  = 0;

static bool cb_tx_ready(void *arg, spi_slave_hd_event_t *event, BaseType_t *awoken);
static bool cb_rx_ready(void *arg, spi_slave_hd_event_t *event, BaseType_t *awoken);
static bool cb_cmd9_recv(void *arg, spi_slave_hd_event_t *event, BaseType_t *awoken);

static interface_handle_t * esp_spi_hd_init(void);
static int esp_spi_hd_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle);
static int32_t esp_spi_hd_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static void esp_spi_hd_deinit(interface_handle_t * handle);
static esp_err_t esp_spi_hd_reset(interface_handle_t *handle);

if_ops_t if_ops = {
	.init = esp_spi_hd_init,
	.write = esp_spi_hd_write,
	.read = esp_spi_hd_read,
	.reset = esp_spi_hd_reset,
	.deinit = esp_spi_hd_deinit,
};

static struct hosted_mempool * buf_mp_tx_g;
static struct hosted_mempool * buf_mp_rx_g;
static struct hosted_mempool * trans_tx_g;
static struct hosted_mempool * trans_rx_g;
static SemaphoreHandle_t mempool_tx_sem = NULL; // to count number of Tx bufs in IDF SPI HD driver

uint8_t power_save_started;
#define IS_HOST_POWER_SAVING() (power_save_started)

static inline void spi_hd_mempool_create(void)
{
	buf_mp_tx_g = hosted_mempool_create(NULL, 0,
			TX_MEMPOOL_NUM_BLOCKS, SPI_HD_BUFFER_SIZE);
	trans_tx_g = hosted_mempool_create(NULL, 0,
			TX_MEMPOOL_NUM_BLOCKS, sizeof(spi_slave_hd_data_t));
	buf_mp_rx_g = hosted_mempool_create(NULL, 0,
			RX_MEMPOOL_NUM_BLOCKS, SPI_HD_BUFFER_SIZE);
	trans_rx_g = hosted_mempool_create(NULL, 0,
			RX_MEMPOOL_NUM_BLOCKS, sizeof(spi_slave_hd_data_t));
#if CONFIG_ESP_CACHE_MALLOC
	assert(buf_mp_tx_g);
	assert(buf_mp_rx_g);
	assert(trans_tx_g);
	assert(trans_rx_g);
#endif
}

static inline void spi_hd_mempool_destroy(void)
{
	hosted_mempool_destroy(buf_mp_tx_g);
	hosted_mempool_destroy(buf_mp_rx_g);
	hosted_mempool_destroy(trans_tx_g);
	hosted_mempool_destroy(trans_rx_g);
}

static inline void *spi_hd_buffer_tx_alloc(size_t nbytes, uint need_memset)
{
	return hosted_mempool_alloc(buf_mp_tx_g, nbytes, need_memset);
}

static inline void spi_hd_buffer_tx_free(void *buf)
{
	hosted_mempool_free(buf_mp_tx_g, buf);
}

static inline void *spi_hd_buffer_rx_alloc(uint need_memset)
{
	return hosted_mempool_alloc(buf_mp_rx_g, SPI_HD_BUFFER_SIZE, need_memset);
}

static inline void spi_hd_buffer_rx_free(void *buf)
{
	hosted_mempool_free(buf_mp_rx_g, buf);
}

static inline spi_slave_hd_data_t *spi_hd_trans_tx_alloc(uint need_memset)
{
	return hosted_mempool_alloc(trans_tx_g, sizeof(spi_slave_hd_data_t), need_memset);
}

static inline void spi_hd_trans_tx_free(spi_slave_hd_data_t *trans)
{
	hosted_mempool_free(trans_tx_g, trans);
}

static inline spi_slave_hd_data_t *spi_hd_trans_rx_alloc(uint need_memset)
{
	return hosted_mempool_alloc(trans_rx_g, sizeof(spi_slave_hd_data_t), need_memset);
}

static inline void spi_hd_trans_rx_free(spi_slave_hd_data_t *trans)
{
	hosted_mempool_free(trans_rx_g, trans);
}

static bool cb_rx_ready(void *arg, spi_slave_hd_event_t *event, BaseType_t *awoken)
{
	// rx dma buffer ready
	ESP_EARLY_LOGV(TAG, "cb_rx_ready");

	// update count
	rx_ready_buf_num++;
	spi_slave_hd_write_buffer(SPI_HOST, SPI_HD_REG_RX_BUF_LEN,
			(uint8_t *)&rx_ready_buf_num, sizeof(rx_ready_buf_num));

	return true;
}

static bool cb_tx_ready(void *arg, spi_slave_hd_event_t *event, BaseType_t *awoken)
{
	// tx buffer loaded to DMA
	ESP_EARLY_LOGD(TAG, "cb_tx_ready %u (current %u)", event->trans->len, tx_ready_buf_size);
	// save the int mask
	uint32_t int_mask = tx_ready_buf_size & SPI_HD_INT_MASK;

	// update tx count, taking into account the len mask
	tx_ready_buf_size = ((tx_ready_buf_size & SPI_HD_TX_BUF_LEN_MASK)
			+ event->trans->len) & SPI_HD_TX_BUF_LEN_MASK;

	// restore the int mask;
	tx_ready_buf_size |= int_mask;

	spi_slave_hd_write_buffer(SPI_HOST, SPI_HD_REG_TX_BUF_LEN,
			(uint8_t *)&tx_ready_buf_size, sizeof(tx_ready_buf_size));

	// set Data Ready
	set_dataready_gpio();

	return true;
}

static bool cb_cmd9_recv(void *arg, spi_slave_hd_event_t *event, BaseType_t *awoken)
{
	// clear the mask
	tx_ready_buf_size &= SPI_HD_TX_BUF_LEN_MASK;

	ESP_EARLY_LOGD(TAG, "cb_cmd9_recv %u", tx_ready_buf_size);
	// clear Data Ready
	reset_dataready_gpio();

	return true;
}

static void flow_ctrl_task(void* pvParameters)
{
	flow_ctrl_sem = xSemaphoreCreateBinary();
	assert(flow_ctrl_sem);

	for(;;) {
		interface_buffer_handle_t buf_handle = {0};

		xSemaphoreTake(flow_ctrl_sem, portMAX_DELAY);

		if (wifi_flow_ctrl)
			buf_handle.throttle_cmd = H_FLOW_CTRL_ON;
		else
			buf_handle.throttle_cmd = H_FLOW_CTRL_OFF;

		ESP_LOGV(TAG, "flow_ctrl %u", buf_handle.throttle_cmd);
		send_to_host_queue(&buf_handle, PRIO_Q_SERIAL);
	}
}

static void start_rx_data_throttling_if_needed(void)
{
	uint32_t queue_load;
	uint8_t load_percent;

	if (slv_cfg_g.throttle_high_threshold > 0) {

		/* Already throttling, nothing to be done */
		if (slv_state_g.current_throttling)
			return;

		queue_load = uxQueueMessagesWaiting(spi_hd_rx_queue[PRIO_Q_OTHERS]);

		load_percent = (queue_load*100/SPI_HD_QUEUE_SIZE);
		if (load_percent > slv_cfg_g.throttle_high_threshold) {
			slv_state_g.current_throttling = 1;
			wifi_flow_ctrl = 1;
#if ESP_PKT_STATS
		pkt_stats.sta_flowctrl_on++;
#endif
			TRIGGER_FLOW_CTRL();
		}
	}
}

static void stop_rx_data_throttling_if_needed(void)
{
	uint32_t queue_load;
	uint8_t load_percent;

	if (slv_state_g.current_throttling) {

		queue_load = uxQueueMessagesWaiting(spi_hd_rx_queue[PRIO_Q_OTHERS]);


		load_percent = (queue_load*100/SPI_HD_QUEUE_SIZE);
		if (load_percent < slv_cfg_g.throttle_low_threshold) {
			slv_state_g.current_throttling = 0;
			wifi_flow_ctrl = 0;
#if ESP_PKT_STATS
		pkt_stats.sta_flowctrl_off++;
#endif
			TRIGGER_FLOW_CTRL();
		}
	}
}

static void esp_spi_hd_get_bus_cfg(spi_bus_config_t * bus_cfg)
{
	bus_cfg->data0_io_num = GPIO_D0;
	bus_cfg->data1_io_num = GPIO_D1;
#if (NUM_DATA_BITS == 4)
	bus_cfg->data2_io_num = GPIO_D2;
	bus_cfg->data3_io_num = GPIO_D3;
#else
	bus_cfg->data2_io_num = -1;
	bus_cfg->data3_io_num = -1;
#endif
	bus_cfg->sclk_io_num = GPIO_SCLK;
	bus_cfg->max_transfer_sz = SPI_HD_BUFFER_SIZE;
#if (NUM_DATA_BITS == 4)
	bus_cfg->flags = SPICOMMON_BUSFLAG_QUAD;
#else
	bus_cfg->flags = SPICOMMON_BUSFLAG_DUAL;
#endif
	bus_cfg->intr_flags = 0;
}

static void esp_spi_hd_get_slot_cfg(spi_slave_hd_slot_config_t * slot_cfg)
{
	slot_cfg->spics_io_num = GPIO_CS;
	slot_cfg->flags = 0;
	slot_cfg->mode = ESP_SPI_HD_MODE;
	slot_cfg->command_bits = NUM_COMMAND_BITS;
	slot_cfg->address_bits = NUM_ADDRESS_BITS;
	slot_cfg->dummy_bits   = NUM_DUMMY_BITS;
	slot_cfg->queue_size = SPI_HD_QUEUE_SIZE;
	slot_cfg->dma_chan = DMA_CHAN;
	slot_cfg->cb_config = (spi_slave_hd_callback_config_t) {
		.cb_send_dma_ready = cb_tx_ready,  // triggered when Tx buffer is loaded to DMA
		.cb_recv_dma_ready = cb_rx_ready,  // triggered when Rx buffer has data from DMA
		.cb_cmd9           = cb_cmd9_recv, // triggered when Cmd9 is received
	};
}

static int esp_spi_hd_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	if (!if_handle || (if_handle->state != ACTIVE) || !buf_handle) {
		ESP_LOGE(TAG, "%s: Invalid state/args", __func__);
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		return ESP_FAIL;
	}

	xSemaphoreTake(spi_hd_rx_sem, portMAX_DELAY);

	if (pdFALSE == xQueueReceive(spi_hd_rx_queue[PRIO_Q_SERIAL], buf_handle, 0))
		if (pdFALSE == xQueueReceive(spi_hd_rx_queue[PRIO_Q_BT], buf_handle, 0))
			if (pdFALSE == xQueueReceive(spi_hd_rx_queue[PRIO_Q_OTHERS], buf_handle, 0)) {
				ESP_LOGE(TAG, "%s No element in rx queue", __func__);
		return ESP_FAIL;
	}

	stop_rx_data_throttling_if_needed();

	return buf_handle->payload_len;
}

static void spi_hd_read_done(void *handle)
{
	esp_err_t res;

	// spi hd rx transaction and buffer can now be put back into the rx queue
	spi_slave_hd_data_t * trans = (spi_slave_hd_data_t *)handle;
	res = spi_slave_hd_queue_trans(SPI_HOST, SPI_SLAVE_CHAN_RX,
				trans, portMAX_DELAY);
	if (res) {
		ESP_LOGE(TAG, "%s: Failed to re-queue Rx transaction", __func__);
		spi_hd_buffer_rx_free(trans->data);
		spi_hd_trans_rx_free(trans);
	}
}

static void spi_hd_rx_task(void* pvParameters)
{
	int i;
	uint8_t * buf = NULL;
	esp_err_t ret = ESP_OK;
	spi_slave_hd_data_t *rx_trans = NULL;
	spi_slave_hd_data_t *ret_trans = NULL;
	esp_err_t res;
	interface_buffer_handle_t buf_handle = {0};

	ESP_LOGD(TAG, "starting spi_hd_rx_task");

	// prepare buffers and preload rx transactions
	for (i = 0; i < SPI_HD_QUEUE_SIZE; i++) {
		buf = spi_hd_buffer_rx_alloc(MEMSET_REQUIRED);
		rx_trans = spi_hd_trans_rx_alloc(MEMSET_REQUIRED);
		rx_trans->data = buf;
		rx_trans->len  = SPI_HD_BUFFER_SIZE;
		res = spi_slave_hd_queue_trans(SPI_HOST, SPI_SLAVE_CHAN_RX,
				rx_trans, portMAX_DELAY);
		if (res) {
			ESP_LOGE(TAG, "Failed to queue Rx transaction %"PRIu16, i);
			spi_hd_buffer_rx_free(buf);
			spi_hd_trans_rx_free(rx_trans);
		}
	}

	// delay for a while to let app main threads start and become ready
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// spi hd now ready: open data path
	if (context.event_handler) {
		ESP_LOGD(TAG, "Trigger open data path at slave");
		context.event_handler(ESP_OPEN_DATA_PATH);
	} else {
		ESP_LOGW(TAG, "No event handler, skipping open data path");
	}

	while (1) {

		// wait for incoming transactions
		res = spi_slave_hd_get_trans_res(SPI_HOST, SPI_SLAVE_CHAN_RX,
				&ret_trans, portMAX_DELAY);
		if (ret) {
			ESP_LOGV(TAG, "spi_slave_hd_get_trans_res returned failure");
			continue;
		}

		// process incoming data
		// received data len is in spi_slave_hd_data_t.trans_len
		if (!ret_trans->trans_len) {
			ESP_LOGE(TAG, "spi_slave_hd_get_trans_res returned 0 len");
			spi_hd_read_done(ret_trans); // return the transaction back to the rx queue
			continue;
		}

		/* Process received data */
		buf_handle.payload = ret_trans->data;
		buf_handle.payload_len = ret_trans->trans_len;

		UPDATE_HEADER_RX_PKT_NO((struct esp_payload_header *)buf_handle.payload);

		/* Decode via frame component */
		{
			eh_frame_result_t fres = eh_frame_decode(
				buf_handle.payload, buf_handle.payload_len, &buf_handle);
			if (fres == EH_FRAME_DUMMY || fres != EH_FRAME_OK) {
				if (fres != EH_FRAME_DUMMY)
					ESP_LOGE(TAG, "spi_hd_rx: frame_decode err %d, drop", fres);
				spi_hd_read_done(ret_trans);
				continue;
			}
		}

		if (buf_handle.flags & FLAG_POWER_SAVE_STARTED) {
			ESP_LOGI(TAG, "Host informed starting to power sleep");
			power_save_started = 1;
			if (context.event_handler)
				context.event_handler(ESP_POWER_SAVE_ON);
		} else if (buf_handle.flags & FLAG_POWER_SAVE_STOPPED) {
			ESP_LOGI(TAG, "Host informed that it waken up");
			tx_ready_buf_size = 0;
			power_save_started = 0;
			if (context.event_handler)
				context.event_handler(ESP_POWER_SAVE_OFF);
		}

		buf_handle.free_buf_handle = spi_hd_read_done;
		buf_handle.spi_hd_trans_handle = ret_trans;

		start_rx_data_throttling_if_needed();

#if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.hs_bus_sta_in++;
#endif
		if (buf_handle.if_type == ESP_SERIAL_IF) {
			xQueueSend(spi_hd_rx_queue[PRIO_Q_SERIAL], &buf_handle, portMAX_DELAY);
		} else if (buf_handle.if_type == ESP_HCI_IF) {
			xQueueSend(spi_hd_rx_queue[PRIO_Q_BT], &buf_handle, portMAX_DELAY);
		} else {
			xQueueSend(spi_hd_rx_queue[PRIO_Q_OTHERS], &buf_handle, portMAX_DELAY);
		}

		xSemaphoreGive(spi_hd_rx_sem);
	}
}

static void spi_hd_tx_done_task(void* pvParameters)
{
	esp_err_t err = ESP_OK;
	spi_slave_hd_data_t *ret_trans;

	while (1) {
		err = spi_slave_hd_get_trans_res(SPI_HOST, SPI_SLAVE_CHAN_TX,
				&ret_trans, portMAX_DELAY);
		if (err == ESP_OK) {
			spi_hd_buffer_tx_free(ret_trans->data);
			spi_hd_trans_tx_free(ret_trans);
			xSemaphoreGive(mempool_tx_sem);
		} else {
			ESP_LOGE(TAG, "error getting completed tx transaction");
			ESP_LOGE(TAG, "error code: %d %s", err, esp_err_to_name(err));
		}
	}
}

static interface_handle_t * esp_spi_hd_init(void)
{
	if (if_handle_g.state >= DEACTIVE) {
		return &if_handle_g;
	}

	esp_err_t ret = ESP_OK;
	uint32_t value = 0;
	uint16_t prio_q_idx = 0;
	uint8_t init_value[SOC_SPI_MAXIMUM_BUFFER_SIZE] = {0x0}; // used to init SPI shared registers

	spi_bus_config_t bus_cfg;
	spi_slave_hd_slot_config_t slave_hd_cfg;

	/* Configuration for data_ready line */
	gpio_config_t io_data_ready_conf={
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = GPIO_MASK_DATA_READY
	};

	// get SPI HD bus and slot configurations
	esp_spi_hd_get_bus_cfg(&bus_cfg);
	esp_spi_hd_get_slot_cfg(&slave_hd_cfg);

	/* Configure data_ready line as output */
	gpio_config(&io_data_ready_conf);
	reset_dataready_gpio();

	/* Enable pull-ups on SPI lines
	 * so that no rogue pulses when no master is connected
	 */
	gpio_set_pull_mode(GPIO_DATA_READY, H_DR_PULL_REGISTER);
	gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

	ESP_LOGI(TAG, "SPI HD Host:%"PRIu16" mode: %"PRIu16 ", Freq:ConfigAtHost",
			SPI_HOST, slave_hd_cfg.mode);
#if (NUM_DATA_BITS == 4)
	ESP_LOGI(TAG, "SPI HD GPIOs: Dat0: %"PRIu16 ", Dat1: %"PRIu16 ", Dat2: %"PRIu16
			", Dat3: %"PRIu16 ", CS: %"PRIu16 ", CLK: %"PRIu16 ", Data Ready: %"PRIu16 ,
			GPIO_D0, GPIO_D1, GPIO_D2, GPIO_D3, GPIO_CS, GPIO_SCLK, GPIO_DATA_READY);
#else
	ESP_LOGI(TAG, "SPI HD GPIOs: Dat0: %"PRIu16 ", Dat1: %"PRIu16
			", CS: %"PRIu16 ", CLK: %"PRIu16 ", Data Ready: %"PRIu16 ,
			GPIO_D0, GPIO_D1, GPIO_CS, GPIO_SCLK, GPIO_DATA_READY);
#endif
	ESP_LOGI(TAG, "Hosted SPI HD queue size:%"PRIu16, SPI_HD_QUEUE_SIZE);

#if !H_DATAREADY_ACTIVE_HIGH
	ESP_LOGI(TAG, "DataReady: Active Low");
#else
	ESP_LOGI(TAG, "DataReady: Active High");
#endif

	/* Initialize SPI slave interface */

	ret = spi_slave_hd_init(SPI_HOST, &bus_cfg, &slave_hd_cfg);
	assert(ret == ESP_OK);

	// initialise shared spi hd registers

	// Reset all the SPI shared registers to 0
	spi_slave_hd_write_buffer(SPI_HOST, 0, init_value, SOC_SPI_MAXIMUM_BUFFER_SIZE);

	// set our Max Tx/Rx buffer size
	// host can use this to determine max size of data to transfer
	value = SPI_HD_BUFFER_SIZE;
	spi_slave_hd_write_buffer(SPI_HOST, SPI_HD_REG_MAX_TX_BUF_LEN,
			(uint8_t *)&value, sizeof(value));

	value = SPI_HD_BUFFER_SIZE;
	spi_slave_hd_write_buffer(SPI_HOST, SPI_HD_REG_MAX_RX_BUF_LEN,
			(uint8_t *)&value, sizeof(value));

	// SPI HD SLave is ready
	value = SPI_HD_STATE_SLAVE_READY;
	spi_slave_hd_write_buffer(SPI_HOST, SPI_HD_REG_SLAVE_READY,
			(uint8_t *)&value, sizeof(value));

	// wait for data path to be opened by host before completing init
	ESP_LOGI(TAG, "Waiting for Host to open data path...");
	while (1) {
		spi_slave_hd_read_buffer(SPI_HOST, SPI_HD_REG_SLAVE_CTRL, (uint8_t *)&value, sizeof(value));
		if (value & SPI_HD_CTRL_DATAPATH_ON) {
			ESP_LOGI(TAG, "Host has opened data path");
			break;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	spi_hd_mempool_create();
	mempool_tx_sem = xSemaphoreCreateCounting(SPI_HD_QUEUE_SIZE, SPI_HD_QUEUE_SIZE);
	assert(mempool_tx_sem);

	spi_hd_rx_sem = xSemaphoreCreateCounting(SPI_HD_QUEUE_SIZE * MAX_PRIORITY_QUEUES, 0);
	assert(spi_hd_rx_sem != NULL);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		spi_hd_rx_queue[prio_q_idx] = xQueueCreate(SPI_HD_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_hd_rx_queue[prio_q_idx] != NULL);
	}

	assert(xTaskCreate(spi_hd_rx_task, "spi_hd_rx_task" ,
			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);

	// task to clean up after doing tx
	assert(xTaskCreate(spi_hd_tx_done_task, "spi_hd_tx_done_task" ,
			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);

	assert(xTaskCreate(flow_ctrl_task, "flow_ctrl_task" ,
			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);

	// data path opened. Continue
	memset(&if_handle_g, 0, sizeof(if_handle_g));
	if_handle_g.state = ACTIVE;

	return &if_handle_g;
}

static void esp_spi_hd_deinit(interface_handle_t * handle)
{
#if H_PS_UNLOAD_BUS_WHILE_PS

	if (if_handle_g.state == DEINIT) {
		ESP_LOGW(TAG, "SPI HD already deinitialized");
		return;
	}
	if_handle_g.state = DEINIT;
	spi_hd_mempool_destroy();
	vSemaphoreDelete(mempool_tx_sem);
	mempool_tx_sem = NULL;

	// close data path
	if (context.event_handler) {
		context.event_handler(ESP_CLOSE_DATA_PATH);
	}

	assert(spi_slave_hd_deinit(SPI_HOST) == ESP_OK);
#endif
}

static esp_err_t esp_spi_hd_reset(interface_handle_t *handle)
{
	spi_bus_config_t bus_cfg;
	spi_slave_hd_slot_config_t slave_hd_cfg;
	esp_err_t ret = ESP_OK;

	ret = spi_slave_hd_deinit(SPI_HOST);
	if (ret != ESP_OK)
		return ret;

	esp_spi_hd_get_bus_cfg(&bus_cfg);
	esp_spi_hd_get_slot_cfg(&slave_hd_cfg);
	ret = spi_slave_hd_init(SPI_HOST, &bus_cfg, &slave_hd_cfg);
	if (ret != ESP_OK)
		return ret;

	return ret;
}

static int32_t esp_spi_hd_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;
	int32_t total_len = 0;
	uint8_t* sendbuf = NULL;
	spi_slave_hd_data_t *tx_trans = NULL;

	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments");
		return ESP_FAIL;
	}

	if (handle->state != ACTIVE) {
		return ESP_FAIL;
	}

	if (!buf_handle->throttle_cmd) {
		// skip this check for flow control packets (they don't have a payload)
		if (!buf_handle->payload_len || !buf_handle->payload){
			ESP_LOGE(TAG , "Invalid arguments, len:%"PRIu16, buf_handle->payload_len);
			return ESP_FAIL;
		}
	}

	if (IS_HOST_POWER_SAVING()) {
		ESP_LOGE(TAG, "Host is power saving, skipping write");
		return ESP_FAIL;
	}

	total_len = buf_handle->payload_len + eh_frame_hdr_size();

	xSemaphoreTake(mempool_tx_sem, portMAX_DELAY);
	sendbuf = spi_hd_buffer_tx_alloc(total_len, MEMSET_REQUIRED);
	if (sendbuf == NULL) {
		ESP_LOGE(TAG , "send buffer[%"PRIu32"] malloc fail", total_len);
		MEM_DUMP("malloc failed");
		return ESP_FAIL;
	}

	/* Encode wire header via frame component */
	{
		interface_buffer_handle_t h = {0};
		h.if_type        = buf_handle->if_type;
		h.if_num         = buf_handle->if_num;
		h.flags          = buf_handle->flags;
		h.seq_num        = buf_handle->seq_num;
		h.throttle_cmd   = buf_handle->throttle_cmd;
		UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
		eh_frame_encode(sendbuf, &h, buf_handle->payload_len);
	}

	memcpy(sendbuf + eh_frame_hdr_size(), buf_handle->payload, buf_handle->payload_len);

	ESP_LOGD(TAG, "sending %"PRIu32 " bytes, flag: 0x%02x", total_len, buf_handle->flags);
	ESP_HEXLOGD("spi_hd_tx", sendbuf, total_len, 32);

	tx_trans = spi_hd_trans_tx_alloc(MEMSET_REQUIRED);
	tx_trans->data = sendbuf;
	tx_trans->len  = total_len;

	ret = spi_slave_hd_queue_trans(SPI_HOST, SPI_SLAVE_CHAN_TX,
				tx_trans, portMAX_DELAY);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "spi hd slave transmit error, ret : 0x%"PRIx16, ret);
		spi_hd_buffer_tx_free(sendbuf);
		spi_hd_trans_tx_free(tx_trans);
		return ESP_FAIL;
	}

#if ESP_PKT_STATS
	if (buf_handle->if_type == ESP_STA_IF)
		pkt_stats.sta_sh_out++;
	else if (buf_handle->if_type == ESP_SERIAL_IF)
		pkt_stats.serial_tx_total++;
#endif

	return buf_handle->payload_len;
}

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	memset(&context, 0, sizeof(context));

	context.type = SPI_HD;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

void generate_startup_event(uint8_t cap, uint32_t ext_cap, uint8_t raw_tp_cap,
                            const uint32_t feat_caps[8])
{
	interface_buffer_handle_t buf_handle = {0};
	struct esp_priv_event *event = NULL;
	uint16_t len = 0;
	uint32_t total_len = 0;
	spi_slave_hd_data_t *tx_trans = NULL;
	esp_err_t ret;
	int rc;

	/* Init frame component for CP SPI-HD */
	{
		eh_frame_cfg_t frame_cfg = EH_FRAME_CFG_CP_FG_LINUX_SPI_HD_DEFAULT;
		eh_frame_init(&frame_cfg);
	}

	xSemaphoreTake(mempool_tx_sem, portMAX_DELAY);
	buf_handle.payload = spi_hd_buffer_tx_alloc(512, MEMSET_REQUIRED);
	assert(buf_handle.payload);

	event = (struct esp_priv_event *) (buf_handle.payload + eh_frame_hdr_size());
	event->event_type = eh_priv_event_init_wire();

	/* Build TLV payload via eh_tlv pack APIs */
	eh_tlv_builder_t tlv;
	uint16_t budget = MAX_TRANSPORT_BUF_SIZE - eh_frame_hdr_size() - 2;
	eh_tlv_builder_init(&tlv, event->event_data, budget);

#if EH_TLV_V1_LINUX
	{
		struct eh_fw_version fw_ver = { 0 };
		strlcpy(fw_ver.project_name, PROJECT_NAME, sizeof(fw_ver.project_name));
		fw_ver.major    = PROJECT_VERSION_MAJOR_1;
		fw_ver.minor    = PROJECT_VERSION_MINOR_1;
		fw_ver.patch    = PROJECT_VERSION_PATCH_1;

		rc = eh_tlv_pack_v1_linux(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			&fw_ver, sizeof(fw_ver));
		if (rc) { ESP_LOGE(TAG, "TLV v1_linux overflow"); spi_hd_buffer_tx_free(buf_handle.payload); xSemaphoreGive(mempool_tx_sem); return; }
	}
#endif

#if EH_TLV_V1_MCU
	{
		uint32_t fw_version = EH_VERSION_VAL(
			PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MINOR_1, PROJECT_VERSION_PATCH_1);

		rc = eh_tlv_pack_v1_mcu(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			ext_cap, fw_version,
			SPI_HD_QUEUE_SIZE, SPI_HD_QUEUE_SIZE);
		if (rc) { ESP_LOGE(TAG, "TLV v1_mcu overflow"); spi_hd_buffer_tx_free(buf_handle.payload); xSemaphoreGive(mempool_tx_sem); return; }
	}
#endif

#if EH_TLV_V2
	rc = eh_tlv_pack_v2(&tlv,
		feat_caps, EH_FEAT_CAPS_COUNT,
		ESP_HOSTED_HDR_VERSION_V2, ESP_HOSTED_RPC_VERSION_V2,
		RPC_EP_NAME_REQ, RPC_EP_NAME_EVT);
	if (rc) { ESP_LOGE(TAG, "TLV v2 overflow"); spi_hd_buffer_tx_free(buf_handle.payload); xSemaphoreGive(mempool_tx_sem); return; }
#endif

	len = eh_tlv_builder_len(&tlv);
	event->event_len = len;
	len += 2;

	/* Encode wire header via frame component */
	{
		interface_buffer_handle_t h = {0};
		h.if_type  = ESP_PRIV_IF;
		h.if_num   = 0;
		h.pkt_type = eh_priv_pkt_type_event_wire();
		UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
		eh_frame_encode(buf_handle.payload, &h, len);
	}

	total_len = len + eh_frame_hdr_size();

	if (!IS_SPI_HD_DMA_ALIGNED(total_len)) {
		MAKE_SPI_HD_DMA_ALIGNED(total_len);
	}

	buf_handle.payload_len = total_len;

	tx_trans = spi_hd_trans_tx_alloc(MEMSET_REQUIRED);
	tx_trans->data = buf_handle.payload;
	tx_trans->len  = buf_handle.payload_len;

	ret = spi_slave_hd_queue_trans(SPI_HOST, SPI_SLAVE_CHAN_TX,
				tx_trans, portMAX_DELAY);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "statup: spi hd slave transmit error, ret : 0x%"PRIx16, ret);
		spi_hd_buffer_tx_free(buf_handle.payload);
		spi_hd_trans_tx_free(tx_trans);
	}
}
