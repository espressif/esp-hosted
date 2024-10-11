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

#include <stdlib.h>

#include "esp_memory_utils.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "hal/spi_ll.h"

#include "common.h"
#include "esp_hosted_config.h"
#include "os_wrapper.h"
#include "spi_hd_wrapper.h"

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
/* Use DMA Aligned Buffers for reg reads, buf read / writes */
#define USE_DMA_ALIGNED_BUF (1)
#else
#define USE_DMA_ALIGNED_BUF (0)
#endif

#ifdef CONFIG_IDF_TARGET_ESP32P4
/* Enable workaround if got SPI Read Errors on ESP32-P4 due to caching */
#define SPI_WORKAROUND (0)
#else
#define SPI_WORKAROUND (0)
#endif

#if SPI_WORKAROUND
#include "esp_cache.h"
#endif

/* SPI_WORKAROUND requires DMA Aligned Buffers to be used */
#if SPI_WORKAROUND && !USE_DMA_ALIGNED_BUF
#error SPI_WORKAROUND and USE_DMA_ALIGNED_BUF must be enabled together
#endif

#include "esp_log.h"
static const char TAG[] = "spi_hd_wrapper";

#define MASTER_HOST  SPI2_HOST // only SPI2 can be used in Half-duplex mode
#define DMA_CHAN     SPI_DMA_CH_AUTO

#ifndef MIN
#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#endif

#define SPI_QUAD 1
#define SPI_DUAL 0
#define SPI_MONO 0

// num data lines used in each phase: cmd = 1, addr = 4, dummy = 1, data = 4
#define SPI_QUAD_FLAGS (SPI_TRANS_MODE_QIO | SPI_TRANS_MULTILINE_ADDR)

// num data lines used in each phase: cmd = 1, addr = 2, dummy = 1, data = 2
#define SPI_DUAL_FLAGS (SPI_TRANS_MODE_DIO | SPI_TRANS_MULTILINE_ADDR)

#define SPI_HD_FAIL_IF_NULL(x) do { \
		if (!x) return ESP_FAIL;  \
	} while (0);

#define SPI_HD_LOCK(x) do { \
	if (x) g_h.funcs->_h_lock_mutex(spi_hd_bus_lock, portMAX_DELAY); \
} while (0);

#define SPI_HD_UNLOCK(x) do { \
	if (x) g_h.funcs->_h_unlock_mutex(spi_hd_bus_lock); \
} while (0);

// spi_hd context structure
typedef struct spi_hd_ctx_t {
	spi_device_handle_t handle;
} spi_hd_ctx_t;

static spi_hd_ctx_t * ctx = NULL;
static void * spi_hd_bus_lock;

#if USE_DMA_ALIGNED_BUF
/* we use 64-bit DMA aligned buffer for reading register data */

#define DMA_ALIGNED_BUF_LEN 64 // ESP32-P4 requires 64 byte aligned buffers
DRAM_DMA_ALIGNED_ATTR static uint8_t dma_data_buf[DMA_ALIGNED_BUF_LEN];
#endif

// initially we start off using 2 data lines
static uint32_t spi_hd_rx_tx_flags = SPI_DUAL_FLAGS;

// returns the spi command to send based on the provided mode flags
static uint16_t spi_hd_get_hd_command(spi_command_t cmd_t, uint32_t flags)
{
	spi_line_mode_t line_mode = {
		.cmd_lines = 1,
	};

	if (flags & SPI_TRANS_MODE_DIO) {
		line_mode.data_lines = 2;
		if (flags & SPI_TRANS_MODE_DIOQIO_ADDR) {
			line_mode.addr_lines = 2;
		} else {
			line_mode.addr_lines = 1;
		}
	} else if (flags & SPI_TRANS_MODE_QIO) {
		line_mode.data_lines = 4;
		if (flags & SPI_TRANS_MODE_DIOQIO_ADDR) {
			line_mode.addr_lines = 4;
		} else {
			line_mode.addr_lines = 1;
		}
	} else {
		line_mode.data_lines = 1;
		line_mode.addr_lines = 1;
	}

	return spi_ll_get_slave_hd_command(cmd_t, line_mode);
}

// get number of dummy bits for the SPI transaction
static int spi_hd_get_hd_dummy_bits(uint32_t flags)
{
	spi_line_mode_t line_mode = {};

	if (flags & SPI_TRANS_MODE_DIO) {
		line_mode.data_lines = 2;
	} else if (flags & SPI_TRANS_MODE_QIO) {
		line_mode.data_lines = 4;
	} else {
		line_mode.data_lines = 1;
	}

	return spi_ll_get_slave_hd_dummy_bits(line_mode);
}

static esp_err_t spi_hd_read_reg(uint32_t addr, uint8_t *out_data, int len, uint32_t flags)
{
	spi_transaction_ext_t t = {
		.base = {
			.cmd = spi_hd_get_hd_command(SPI_CMD_HD_RDBUF, flags),
			.addr = addr % 72,
			.rxlength = len * 8,
			.rx_buffer = out_data,
			.flags = flags | SPI_TRANS_VARIABLE_DUMMY,
		},
		.dummy_bits = spi_hd_get_hd_dummy_bits(flags),
	};

#if USE_DMA_ALIGNED_BUF
	/* tell lower layer that we have manually aligned buffer for dma */
	t.base.flags |= SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;
#endif

	return spi_device_transmit(ctx->handle, (spi_transaction_t *)&t);
}

static esp_err_t spi_hd_wrcmd9(uint32_t flags)
{
	spi_transaction_t t = {
		.cmd = spi_hd_get_hd_command(SPI_CMD_HD_INT1, flags),
		.flags = flags,
	};

	return spi_device_transmit(ctx->handle, &t);
}

static esp_err_t spi_hd_write_reg(const uint8_t *data, int addr, int len, uint32_t flags)
{
	spi_transaction_ext_t t = {
		.base = {
			.cmd = spi_hd_get_hd_command(SPI_CMD_HD_WRBUF, flags),
			.addr = addr % 72,
			.length = len * 8,
			.tx_buffer = data,
			.flags = flags | SPI_TRANS_VARIABLE_DUMMY,
		},
		.dummy_bits = spi_hd_get_hd_dummy_bits(flags),
	};

	return spi_device_transmit(ctx->handle, (spi_transaction_t *)&t);
}

static esp_err_t spi_hd_rddma_seg(uint8_t *out_data, int seg_len, uint32_t flags)
{
#if USE_DMA_ALIGNED_BUF
	/* Note: this only works if data is read in one segment, which is
	 * what is currently done
	 * incoming mempool allocated buffer's actual size is MAX_SPI_HD_BUFFER_SIZE,
	 * so this padded length should be okay */
	uint32_t padded_len = ((seg_len + DMA_ALIGNED_BUF_LEN - 1) / DMA_ALIGNED_BUF_LEN) * DMA_ALIGNED_BUF_LEN;
#endif

#if SPI_WORKAROUND
	/* this ensures RX DMA data in cache is sync to memory */
	assert(ESP_OK == esp_cache_msync((void *)out_data, padded_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M));
#endif

	spi_transaction_ext_t t = {
		.base = {
			.cmd = spi_hd_get_hd_command(SPI_CMD_HD_RDDMA, flags),
#if USE_DMA_ALIGNED_BUF
			.rxlength = padded_len * 8,
			/* tell lower layer that we have manually aligned buffer for dma */
			.flags = flags | (SPI_TRANS_VARIABLE_DUMMY | SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL),
#else
			.rxlength = seg_len * 8,
			.flags = flags | SPI_TRANS_VARIABLE_DUMMY,
#endif
			.rx_buffer = out_data,
		},
		.dummy_bits = spi_hd_get_hd_dummy_bits(flags),
	};

	return spi_device_transmit(ctx->handle, (spi_transaction_t *)&t);
}

static esp_err_t spi_hd_rddma_done(uint32_t flags)
{
	spi_transaction_t end_t = {
		.cmd = spi_hd_get_hd_command(SPI_CMD_HD_INT0, flags),
		.flags = flags,
	};
	return spi_device_transmit(ctx->handle, &end_t);
}

static esp_err_t spi_hd_rddma(uint8_t *out_data, int len, int seg_len, uint32_t flags)
{
	if (!esp_ptr_dma_capable(out_data) || ((intptr_t)out_data % 4) != 0) {
		return ESP_ERR_INVALID_ARG;
	}
	seg_len = (seg_len > 0) ? seg_len : len;

	uint8_t *read_ptr = out_data;
	esp_err_t ret = ESP_OK;
	while (len > 0) {
		int send_len = MIN(seg_len, len);

		ret = spi_hd_rddma_seg(read_ptr, send_len, flags);
		if (ret != ESP_OK) {
			return ret;
		}

		len -= send_len;
		read_ptr += send_len;
	}
	return spi_hd_rddma_done(flags);
}

static esp_err_t spi_hd_wrdma_seg(const uint8_t *data, int seg_len, uint32_t flags)
{
#if USE_DMA_ALIGNED_BUF
	/* Note: this only works if data is written in one segment, which is
	 * what is currently done
	 * incoming mempool allocated buffer's actual size is MAX_SPI_HD_BUFFER_SIZE,
	 * so this padded length should be okay */
	uint32_t padded_len = ((seg_len + DMA_ALIGNED_BUF_LEN - 1) / DMA_ALIGNED_BUF_LEN) * DMA_ALIGNED_BUF_LEN;
#endif

	spi_transaction_ext_t t = {
		.base = {
			.cmd = spi_hd_get_hd_command(SPI_CMD_HD_WRDMA, flags),
#if USE_DMA_ALIGNED_BUF
			.length = padded_len * 8,
			/* tell lower layer that we have manually aligned buffer for dma */
			.flags = flags | (SPI_TRANS_VARIABLE_DUMMY | SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL),
#else
			.length = seg_len * 8,
			.flags = flags | SPI_TRANS_VARIABLE_DUMMY,
#endif
			.tx_buffer = data,
		},
		.dummy_bits = spi_hd_get_hd_dummy_bits(flags),
	};
	return spi_device_transmit(ctx->handle, (spi_transaction_t *)&t);
}

static esp_err_t spi_hd_wrdma_done(uint32_t flags)
{
	spi_transaction_t end_t = {
		.cmd = spi_hd_get_hd_command(SPI_CMD_HD_WR_END, flags),
		.flags = flags,
	};
	return spi_device_transmit(ctx->handle, &end_t);
}

static esp_err_t spi_hd_wrdma(const uint8_t *data, int len, int seg_len, uint32_t flags)
{
	if (!esp_ptr_dma_capable(data)) {
		return ESP_ERR_INVALID_ARG;
	}
	seg_len = (seg_len > 0) ? seg_len : len;

	while (len > 0) {
		int send_len = MIN(seg_len, len);

		esp_err_t ret = spi_hd_wrdma_seg(data, send_len, flags);
		if (ret != ESP_OK) {
			return ret;
		}

		len -= send_len;
		data += send_len;
	}

	return spi_hd_wrdma_done(flags);
}

void * hosted_spi_hd_init(void)
{
	// initialise bus and device in ctx
	spi_bus_config_t buscfg = {
		.data0_io_num = H_SPI_HD_PIN_D0,
		.data1_io_num = H_SPI_HD_PIN_D1,
#if (H_SPI_HD_HOST_NUM_DATA_LINES == 4)
		.data2_io_num = H_SPI_HD_PIN_D2,
		.data3_io_num = H_SPI_HD_PIN_D3,
#else
		.data2_io_num = -1,
		.data3_io_num = -1,
#endif
		.sclk_io_num = H_SPI_HD_PIN_CLK,
		.max_transfer_sz = MAX_SPI_HD_BUFFER_SIZE,
#if (H_SPI_HD_HOST_NUM_DATA_LINES == 4)
		.flags = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD),
#else
		.flags = (SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_DUAL),
#endif
		.intr_flags = 0,
	};

	spi_device_interface_config_t devcfg = {
#ifdef CONFIG_IDF_TARGET_ESP32P4
		.clock_source = SPI_CLK_SRC_SPLL,
#endif
		.clock_speed_hz = H_SPI_HD_CLK_MHZ * 1000 * 1000,
		.mode = H_SPI_HD_MODE,
		.spics_io_num = H_SPI_HD_PIN_CS,
		.cs_ena_pretrans = 0,
		.cs_ena_posttrans = 0,
		.command_bits = H_SPI_HD_NUM_COMMAND_BITS,
		.address_bits = H_SPI_HD_NUM_ADDRESS_BITS,
		.dummy_bits = H_SPI_HD_NUM_DUMMY_BITS,
		.queue_size = 16,
		.flags = SPI_DEVICE_HALFDUPLEX,
		.duty_cycle_pos = 128, // 50% duty cycle
		.input_delay_ns = 0,
		.pre_cb = NULL,
		.post_cb = NULL,
	};

	ctx = calloc(1, sizeof(spi_hd_ctx_t));
	assert(ctx);

	// initalize bus
	if (spi_bus_initialize(MASTER_HOST, &buscfg, DMA_CHAN)) {
		ESP_LOGE(TAG, "spi_bus_initialize FAILED");
		goto err_bus_initialize;
	}

	// initialize device
	if (spi_bus_add_device(MASTER_HOST, &devcfg, &ctx->handle)) {
		ESP_LOGE(TAG, "spi_bus_add_device FAILED");
		goto err_add_device;
	}

	gpio_set_drive_capability(H_SPI_HD_PIN_CS, GPIO_DRIVE_CAP_3);
	gpio_set_drive_capability(H_SPI_HD_PIN_CLK, GPIO_DRIVE_CAP_3);

	// initialise mutex for bus locking
	spi_hd_bus_lock = g_h.funcs->_h_create_mutex();
	assert(spi_hd_bus_lock);

	return ctx;

 err_add_device:
	spi_bus_free(MASTER_HOST);
	// fallthrough

 err_bus_initialize:
	free(ctx);
	ctx = NULL;

	ESP_LOGE(TAG, "error %s", __func__);
	return NULL;
}

int hosted_spi_hd_deinit(void *ctx)
{
	spi_hd_ctx_t * qsp_ctx;
	spi_device_handle_t * handle;

	g_h.funcs->_h_destroy_mutex(spi_hd_bus_lock);

	if (!ctx)
		return ESP_FAIL;

	qsp_ctx = (spi_hd_ctx_t *)ctx;
	handle  = &qsp_ctx->handle;

	spi_bus_remove_device(*handle);
	spi_bus_free(MASTER_HOST);
	free(ctx);
	ctx = NULL;

	return ESP_OK;
}

int hosted_spi_hd_read_reg(uint32_t reg, uint32_t *data, int poll, bool lock_required)
{
	int res = 0;
	uint32_t read_data;
	uint32_t temp_data;
	int i = 0;

	SPI_HD_FAIL_IF_NULL(ctx);

	SPI_HD_LOCK(lock_required);

#if SPI_WORKAROUND
	/* this ensures RX DMA data in cache is sync to memory */
	assert(ESP_OK == esp_cache_msync((void *)dma_data_buf, DMA_ALIGNED_BUF_LEN, ESP_CACHE_MSYNC_FLAG_DIR_C2M));
#endif

#if USE_DMA_ALIGNED_BUF
	/* use aligned buffer to read data */
	res = spi_hd_read_reg(reg, (uint8_t *)dma_data_buf, DMA_ALIGNED_BUF_LEN, spi_hd_rx_tx_flags);
	read_data = *(uint32_t *)dma_data_buf;
#else
	res = spi_hd_read_reg(reg, (uint8_t *)&read_data, sizeof(uint32_t), spi_hd_rx_tx_flags);
#endif

	if (res != ESP_OK)
		goto err;

	// reread until value is stable
	for (i = 0; i < poll; i++) {
#if SPI_WORKAROUND
		/* this ensures RX DMA data in cache is sync to memory */
		assert(ESP_OK == esp_cache_msync((void *)dma_data_buf, DMA_ALIGNED_BUF_LEN, ESP_CACHE_MSYNC_FLAG_DIR_C2M));
#endif

#if USE_DMA_ALIGNED_BUF
		/* use aligned buffer to read data */
		res = spi_hd_read_reg(reg, (uint8_t *)dma_data_buf, DMA_ALIGNED_BUF_LEN, spi_hd_rx_tx_flags);
		temp_data = *(uint32_t *)dma_data_buf;
#else
		res = spi_hd_read_reg(reg, (uint8_t *)&temp_data, sizeof(uint32_t), spi_hd_rx_tx_flags);
#endif

		if (res != ESP_OK)
			goto err;

		if (temp_data == read_data) {
			break;
		}
		read_data = temp_data;
	}

	if (i && (i == poll)) {
		// we didn't get a stable value at the end
		res = ESP_FAIL;
		goto err;
	}

	*data = read_data;

 err:
	SPI_HD_UNLOCK(lock_required);

	return res;
}

int hosted_spi_hd_write_reg(uint32_t reg, uint32_t *data, bool lock_required)
{
	int res = 0;

	SPI_HD_FAIL_IF_NULL(ctx);

	SPI_HD_LOCK(lock_required);

	res = spi_hd_write_reg((uint8_t *)data, reg, sizeof(uint32_t), spi_hd_rx_tx_flags);

	SPI_HD_UNLOCK(lock_required);

	return res;
}

int hosted_spi_hd_read_dma(uint8_t *data, uint16_t size, bool lock_required)
{
	int res = 0;

	SPI_HD_FAIL_IF_NULL(ctx);

	SPI_HD_LOCK(lock_required);

	res = spi_hd_rddma(data, size, -1, spi_hd_rx_tx_flags);

	SPI_HD_UNLOCK(lock_required);

	return res;
}

int hosted_spi_hd_write_dma(uint8_t *data, uint16_t size, bool lock_required)
{
	int res = 0;

	SPI_HD_FAIL_IF_NULL(ctx);

	SPI_HD_LOCK(lock_required);

	res = spi_hd_wrdma(data, size, -1, spi_hd_rx_tx_flags);

	SPI_HD_UNLOCK(lock_required);

	return res;
}

int hosted_spi_hd_set_data_lines(uint32_t data_lines)
{
	if (data_lines == H_SPI_HD_CONFIG_2_DATA_LINES) {
		ESP_LOGI(TAG, "use 2 data lines");
		spi_hd_rx_tx_flags = SPI_DUAL_FLAGS;
	} else
	if (data_lines == H_SPI_HD_CONFIG_4_DATA_LINES) {
		ESP_LOGI(TAG, "use 4 data lines");
		spi_hd_rx_tx_flags = SPI_QUAD_FLAGS;
	} else {
		return ESP_FAIL;
	}

	return ESP_OK;
}

int hosted_spi_hd_send_cmd9(void)
{
	int res = 0;

	SPI_HD_FAIL_IF_NULL(ctx);

	res = spi_hd_wrcmd9(spi_hd_rx_tx_flags);

	return res;
}
