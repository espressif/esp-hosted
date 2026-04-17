// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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

/*
 * Registry Dispatch Overlay
 * ─────────────────────────
 * This file keeps protocomm as the underlying TLV framing / transport
 * substrate, but adds a registry dispatch layer on top:
 *
 *   Old per-extension endpoint model (deprecated, kept for compat):
 *       extension → protocomm_add_endpoint("MyEP", handler, priv)
 *       request → protocomm TLV parses "MyEP" → calls handler
 *
 *   New registry dispatch model (active for RPCReq / RPCEvt):
 *       extension → eh_cp_rpc_req_register()  (ext_rpc API)
 *       request → protocomm TLV parses "RPCReq"
 *              → rpc_ll_slist_req_handler()
 *              → esp_proto_extract_msg_id() reads proto field 2 varint
 *              → eh_cp_rpc_dispatch_req() searches registry tables
 *              → calls matching node->handler()
 *
 * The two core endpoints (RPCReq, RPCEvt) are registered during
 * eh_cp_protocomm_init().  Per-extension endpoints are still
 * supported via eh_cp_protocomm_add_endpoint() for backward compat.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_log.h>

#include <protocomm.h>
#include <protocomm_priv.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "eh_transport.h"
#include "eh_common_tlv.h"  /* ESP_HOSTED_RPC_VERSION_V1/V2 */
#include "eh_log.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ll.h"
#include "eh_cp_core.h"   /* proto scanner: eh_proto_extract_msg_id */
#include "eh_cp_master_config.h" /* EH_CP_XXX aliases + RPC_EP_NAME_* */
#if EH_CP_FEAT_RPC_READY

static const char TAG[] = "protocomm_pserial";

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define QUEUE_HANDLE QueueHandle_t
#else
  #define QUEUE_HANDLE xQueueHandle
#endif

#define EPNAME_MAX                   16
#define MIN_EP_NAME_LEN              1
#define MAX_EP_NAME_LEN              (EPNAME_MAX - 1)
/* Queue depth for pserial_task.
 * Must be large enough to absorb the WiFi re-init event storm
 * (ESPInit + sta_stopped + sta_started + sta_connected + network_down/up = 6+
 * events in rapid succession during host-driven wifi_init reconfiguration).
 * ESP32C2 keeps a smaller value due to tighter SRAM. */
#if defined(CONFIG_IDF_TARGET_ESP32C2)
  #define REQ_Q_MAX                  6
#else
  #define REQ_Q_MAX                  20
#endif

#define SIZE_OF_TYPE                  1
#define SIZE_OF_LENGTH                2

#define PROTO_PSER_TLV_T_EPNAME       1
#define PROTO_PSER_TLV_T_DATA         2

#define MAX_SERIAL_DATA_SIZE          (16 * 1024)


typedef esp_err_t (*pserial_xmit)(uint8_t *buf, ssize_t len);
typedef ssize_t (*pserial_recv)(uint8_t *buf, ssize_t len);

static char g_rpc_req_ep[EPNAME_MAX] = RPC_EP_NAME_REQ;
static char g_rpc_evt_ep[EPNAME_MAX] = RPC_EP_NAME_EVT;


struct pserial_config {
	pserial_xmit    xmit;
	pserial_recv    recv;
	QUEUE_HANDLE    req_queue;
};

typedef struct {
	int len;
	uint8_t *data;
	int msg_id;
	int type;
	const char *epname;
} serial_arg_t;


/* Simplified global state */
static protocomm_t *g_protocomm_instance = NULL;
static protocomm_write_ll_cb_t g_write_cb = NULL;
static protocomm_read_ll_cb_t g_read_cb = NULL;
static bool g_protocomm_initialized = false;

/* Dynamic endpoint registry (legacy add_endpoint API only) */
typedef struct eh_cp_endpoint_entry {
    char *name;
    protocomm_req_handler_t handler;
    void *priv_data;
} eh_cp_endpoint_entry_t;

static eh_cp_endpoint_entry_t *g_endpoint_list = NULL;
static size_t g_endpoint_count = 0;
static size_t g_endpoint_cap   = 0;
static SemaphoreHandle_t g_endpoint_mutex = NULL;


// Helper function to safely read 16-bit value (little-endian)
static uint16_t read_le16(const uint8_t *buf) {
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

// Helper function to safely write 16-bit value (little-endian)
static void write_le16(uint8_t *buf, uint16_t value) {
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
}

static esp_err_t ensure_endpoint_capacity(size_t needed)
{
    if (needed <= g_endpoint_cap) return ESP_OK;
    size_t new_cap = (g_endpoint_cap == 0) ? 4 : (g_endpoint_cap + 4);
    if (new_cap < needed) new_cap = needed;
    void *new_buf = realloc(g_endpoint_list, new_cap * sizeof(*g_endpoint_list));
    if (!new_buf) return ESP_ERR_NO_MEM;
    g_endpoint_list = (eh_cp_endpoint_entry_t *)new_buf;
    g_endpoint_cap  = new_cap;
    return ESP_OK;
}

/* Forward declarations for endpoint swap */
static esp_err_t rpc_ll_slist_req_handler(uint32_t session_id,
                                          const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen,
                                          void *priv_data);
static esp_err_t rpc_ll_slist_evt_handler(uint32_t session_id,
                                          const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen,
                                          void *priv_data);

static esp_err_t parse_tlv(uint8_t **buf, size_t *total_len,
		int *type, size_t *len, uint8_t **ptr)
{
	uint8_t *b = *buf;
	uint16_t tlv_len;

	if (*total_len < 3) {
		return ESP_FAIL;
	}

	*type = b[0];
	tlv_len = read_le16(b + 1);

	/* Bounds check: value bytes must fit within remaining buffer */
	if ((size_t)tlv_len > *total_len - 3) {
		ESP_LOGE("parse_tlv", "TLV truncated: type=%d val_len=%u remaining=%zu",
		         b[0], tlv_len, *total_len - 3);
		return ESP_FAIL;
	}

	*len = tlv_len;
	*ptr = (uint8_t *) (b + 3);
	*total_len -= (*len + 3);
	*buf = b + 1 + 2 + tlv_len;
	return ESP_OK;
}

static esp_err_t compose_tlv(const char *epname, uint8_t **out, size_t *outlen)
{
	size_t pos = 0;
	size_t ep_len = strlen(epname);
	/*
	 * TLV (Type - Length - Value) structure is as follows:
	 * --------------------------------------------------------------------------------------------
	 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length | Data Value  |
	 * --------------------------------------------------------------------------------------------
	 *
	 *  Bytes used per field as follows:
	 * --------------------------------------------------------------------------------------------
	 *       1        |        2        | Endpoint length |     1     |      2      | Data length |
	 * --------------------------------------------------------------------------------------------
	 */
	size_t buf_len = SIZE_OF_TYPE + SIZE_OF_LENGTH +
		ep_len + SIZE_OF_TYPE + SIZE_OF_LENGTH + *outlen;

	if (buf_len > 10*1024) {
		ESP_LOGE(TAG, "TLV buffer length too large: %d", (int)buf_len);
		return ESP_FAIL;
	}

	uint8_t *buf = (uint8_t *)calloc(1, buf_len);
	if (buf == NULL) {
		ESP_LOGE(TAG,"%s Mem Alloc Failed [%d]bytes", __func__, (int)buf_len);
		return ESP_FAIL;
	}
	buf[pos++] = PROTO_PSER_TLV_T_EPNAME;
	write_le16(&buf[pos], (uint16_t)ep_len);
	pos += 2;
	memcpy(&buf[pos], epname, ep_len);
	pos += ep_len;
	buf[pos++] = PROTO_PSER_TLV_T_DATA;
	write_le16(&buf[pos], (uint16_t)*outlen);
	pos += 2;
	memcpy(&buf[pos], (*out), *outlen);
	free(*out);
	*out = buf;
	*outlen = buf_len;
	return ESP_OK;
}

esp_err_t eh_cp_rpc_set_endpoints(const char *req_ep, int req_ep_size,
                                           const char *evt_ep, int evt_ep_size)
{
    if (!req_ep || !evt_ep) {
        ESP_LOGE(TAG, "Invalid endpoint names: NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (req_ep_size > (int)EPNAME_MAX || evt_ep_size > (int)EPNAME_MAX) {
        ESP_LOGE(TAG, "Endpoint name too long: req=%d evt=%d",
                 req_ep_size, evt_ep_size);
        return ESP_ERR_INVALID_ARG;
    }

    if (g_protocomm_initialized && g_protocomm_instance) {
        /* Swap endpoints atomically under the endpoint mutex */
        if (g_endpoint_mutex) {
            xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);
        }

        if (strcmp(g_rpc_req_ep, req_ep) != 0) {
            protocomm_remove_endpoint(g_protocomm_instance, g_rpc_req_ep);
            (void)protocomm_add_endpoint(g_protocomm_instance, req_ep,
                                         rpc_ll_slist_req_handler, NULL);
        }
        if (strcmp(g_rpc_evt_ep, evt_ep) != 0) {
            protocomm_remove_endpoint(g_protocomm_instance, g_rpc_evt_ep);
            (void)protocomm_add_endpoint(g_protocomm_instance, evt_ep,
                                         rpc_ll_slist_evt_handler, NULL);
        }

        if (g_endpoint_mutex) {
            xSemaphoreGive(g_endpoint_mutex);
        }
    }

    memcpy(g_rpc_req_ep, req_ep, req_ep_size);
    memcpy(g_rpc_evt_ep, evt_ep, evt_ep_size);
    g_rpc_req_ep[req_ep_size - 1] = '\0';
    g_rpc_evt_ep[evt_ep_size - 1] = '\0';
    ESP_LOGI(TAG, "Set RPC endpoints: req=%s evt=%s", g_rpc_req_ep, g_rpc_evt_ep);
    return ESP_OK;
}

const char *eh_cp_rpc_get_req_ep(void) { return g_rpc_req_ep; }
const char *eh_cp_rpc_get_evt_ep(void) { return g_rpc_evt_ep; }

static bool rpc_ep_is_allowed(const char *epname, bool is_req)
{
    if (!epname || !epname[0]) {
        return false;
    }
    if (is_req) {
        return (strcmp(epname, g_rpc_req_ep) == 0);
    }

    return (strcmp(epname, g_rpc_evt_ep) == 0);
}

static esp_err_t protocomm_pserial_ctrl_req_handler(protocomm_t *pc,
		uint8_t *in, size_t in_len)
{
	uint8_t *buf = in;
	size_t total_len = 0, len = 0;
	int type = 0, ret = 0;
	uint8_t *ptr = NULL;

	char epname[EPNAME_MAX] = {0};
	uint8_t *data = NULL;
	size_t data_len = 0;

	uint8_t *out = NULL;
	size_t outlen = 0;
	struct pserial_config *pserial_cfg = NULL;

	total_len = in_len;

	while (parse_tlv(&buf, &total_len, &type, &len, &ptr) == 0) {
		/*ESP_LOGI(TAG, "Parsed type %d len %d", type, len); */
		switch(type) {
			case PROTO_PSER_TLV_T_EPNAME:
				if (len >= EPNAME_MAX - 1) {
					ESP_LOGE(TAG, "EP Name bigger than supported");
					return ESP_FAIL;
				}
				memcpy(epname, ptr, len);
				/*ESP_LOGI(TAG, "Found ep %s", epname); */
				break;
			case PROTO_PSER_TLV_T_DATA:
				data = ptr;
				data_len = len;
				break;
			default:
				ESP_LOGE(TAG, "Invalid type found in the packet");
				return ESP_FAIL;
		}
	}

	if (data == NULL || data_len == 0 || strlen(epname) == 0) {
		ESP_LOGE(TAG, "TLV components not complete for parsing");
		return ESP_FAIL;
	}
	if (!rpc_ep_is_allowed(epname, true)) {
		ESP_LOGW(TAG, "Dropping request for unconfigured endpoint '%s' (allowed: '%s')",
		         epname, g_rpc_req_ep);
		return ESP_FAIL;
	}
	ret = protocomm_req_handle(pc, epname, 0, data,
			data_len, &out, (ssize_t *) &outlen);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error in handling protocomm request %d", ret);
		return ESP_FAIL;
	}

	pserial_cfg = pc->priv;
	if (!pserial_cfg) {
		ESP_LOGE(TAG, "Unexpected. No pserial_cfg found");
		return ESP_FAIL;
	}

	if (!pserial_cfg->xmit) {
		ESP_LOGE(TAG, "No xmit function pointer found");
		return ESP_FAIL;
	}

	ret = compose_tlv(epname, &out, &outlen);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to compose tlv");
		/* MEM-002: out holds the protocomm response; compose_tlv frees it only
		 * on success (via free(*out) + replace).  Free explicitly on failure. */
		free(out);
		return ESP_FAIL;
	}

	ESP_HEXLOGV("serial_tx", out, outlen, 32);
	ret = (pserial_cfg->xmit)(out, (ssize_t) outlen);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to transmit data");
		/* out is consumed by xmit (serial_write_data frees on both success and
		 * failure paths), so no free() here. */
		return ESP_FAIL;
	}
	return ESP_OK;
}

static esp_err_t protocomm_pserial_ctrl_evnt_handler(protocomm_t *pc,
		const char* epname, uint8_t *in, size_t in_len, int msg_id)
{
	int ret = 0;

	uint8_t *out = NULL;
	size_t outlen = 0;
	struct pserial_config *pserial_cfg = NULL;

	if (!rpc_ep_is_allowed(epname, false)) {
		ESP_LOGW(TAG, "Dropping event for unconfigured endpoint '%s' (allowed: '%s')",
		         epname, g_rpc_evt_ep);
		return ESP_FAIL;
	}
	ret = protocomm_req_handle(pc, epname, msg_id,
			in, in_len, &out, (ssize_t *) &outlen);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error in handling protocomm request %d", ret);
		return ESP_FAIL;
	}

	pserial_cfg = pc->priv;
	if (!pserial_cfg) {
		ESP_LOGE(TAG, "Unexpected. No pserial_cfg found");
		return ESP_FAIL;
	}

	if (!pserial_cfg->xmit) {
		ESP_LOGE(TAG, "No xmit function pointer found");
		return ESP_FAIL;
	}

	ret = compose_tlv(epname, &out, &outlen);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to compose tlv");
		/* MEM-002 (evt path): free protocomm response buffer on compose failure */
		free(out);
		return ESP_FAIL;
	}

	ret = (pserial_cfg->xmit)(out, (ssize_t) outlen);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to transmit data");
		/* xmit (serial_write_data) frees on both success and failure — no free here */
		return ESP_FAIL;
	}
	return ESP_OK;
}

static esp_err_t protocomm_pserial_data_ready(protocomm_t *pc, const char *epname,
		uint8_t *in, int len, int msg_id, int type)
{
	struct pserial_config *pserial_cfg = NULL;
	serial_arg_t arg = {0};
	uint8_t *buf = NULL;

	pserial_cfg = (struct pserial_config *) pc->priv;
	if (!pserial_cfg) {
		ESP_LOGE(TAG, "Unexpected. No pserial_cfg found");
		return ESP_FAIL;
	}

	if (len) {
		buf = (uint8_t *)malloc(len);
		if (buf == NULL) {
			ESP_LOGE(TAG,"%s Failed to allocate memory", __func__);
			return ESP_FAIL;
		}
		memcpy(buf, in, len);
	}

	arg.msg_id = msg_id;
	arg.len = len;
	arg.data = buf;
	arg.type = type;
	arg.epname = epname;


	/* Use a bounded timeout instead of portMAX_DELAY.
	 *
	 * BUG-FIX: portMAX_DELAY here causes a recv_task deadlock.
	 * recv_task → process_rx_pkt() → dispatch_rx(SERIAL) → serial_rx cb
	 * → process_req() → protocomm_pserial_data_ready() [this function].
	 * If the queue is full, recv_task blocks here forever waiting for
	 * pserial_task to drain it — but pserial_task can't drain because it is
	 * waiting for SDIO TX to complete, which stalls waiting for the host to
	 * clock out data.  The host SDIO read thread sees no new data and stalls
	 * too.  Result: silent SDIO deadlock after the WiFi re-init event storm.
	 *
	 * Requests (PROTO_REQ_ENDPOINT): called from recv_task — MUST NOT block.
	 *   Use 0-tick (non-blocking); on overflow, drop with a warning.
	 * Events (PROTO_EVT_ENDPOINT): called from mcu_rpc_evt task — a short
	 *   bounded wait (200 ms) is acceptable; drop on saturation.
	 */
	TickType_t timeout = (type == PROTO_REQ_ENDPOINT)
	                         ? 0
	                         : pdMS_TO_TICKS(200);

	if (xQueueSend(pserial_cfg->req_queue, &arg, timeout) != pdTRUE) {
		ESP_LOGW(TAG, "pserial queue full — dropping %s (type=%d)",
		         type == PROTO_REQ_ENDPOINT ? "req" : "evt", type);
		if (buf) {
			free(buf);
			buf = NULL;
		}
		return ESP_FAIL;
	}

	return ESP_OK;
}

static esp_err_t protocomm_rpc_req_ind(protocomm_t *pc, uint8_t * in, int len, int msg_id)
{
	return protocomm_pserial_data_ready(pc, NULL, in, len, msg_id, PROTO_REQ_ENDPOINT);
}

static esp_err_t protocomm_rpc_evt_ind(protocomm_t *pc, const char* epname, uint8_t * in, int len, int msg_id)
{
	return protocomm_pserial_data_ready(pc, epname, in, len, msg_id, PROTO_EVT_ENDPOINT);
}


static esp_err_t protocomm_pserial_add_ep(const char *ep_name,
		protocomm_req_handler_t req_handler, void *priv_data)
{
	if (!ep_name || !req_handler) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!g_endpoint_mutex) {
		g_endpoint_mutex = xSemaphoreCreateMutex();
	}
	xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);

	for (size_t i = 0; i < g_endpoint_count; i++) {
		if (strcmp(g_endpoint_list[i].name, ep_name) == 0) {
			xSemaphoreGive(g_endpoint_mutex);
			ESP_LOGW(TAG, "Endpoint '%s' already exists", ep_name);
			return ESP_ERR_INVALID_STATE;
		}
	}

	if (ensure_endpoint_capacity(g_endpoint_count + 1) != ESP_OK) {
		xSemaphoreGive(g_endpoint_mutex);
		return ESP_ERR_NO_MEM;
	}

	g_endpoint_list[g_endpoint_count].name = strdup(ep_name);
	g_endpoint_list[g_endpoint_count].handler = req_handler;
	g_endpoint_list[g_endpoint_count].priv_data = priv_data;
	g_endpoint_count++;

	xSemaphoreGive(g_endpoint_mutex);
	ESP_LOGD(TAG, "Added endpoint '%s'", ep_name);
	return ESP_OK;
}

static esp_err_t protocomm_pserial_remove_ep(const char *ep_name)
{
	if (!ep_name) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!g_endpoint_mutex) {
		return ESP_ERR_INVALID_STATE;
	}

	xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);
	for (size_t i = 0; i < g_endpoint_count; i++) {
		if (strcmp(g_endpoint_list[i].name, ep_name) == 0) {
			free(g_endpoint_list[i].name);
			if (i < g_endpoint_count - 1) {
				memmove(&g_endpoint_list[i], &g_endpoint_list[i + 1],
				        (g_endpoint_count - i - 1) * sizeof(*g_endpoint_list));
			}
			g_endpoint_count--;
			xSemaphoreGive(g_endpoint_mutex);
			ESP_LOGD(TAG, "Removed endpoint '%s'", ep_name);
			return ESP_OK;
		}
	}
	xSemaphoreGive(g_endpoint_mutex);
	ESP_LOGW(TAG, "Endpoint '%s' not found", ep_name);
	return ESP_ERR_NOT_FOUND;
}

static void pserial_task(void *params)
{
	protocomm_t *pc = (protocomm_t *) params;
	struct pserial_config *pserial_cfg = NULL;
	int len = 0, ret = 0;
	serial_arg_t arg = {0};

	pserial_cfg = (struct pserial_config *) pc->priv;
	if (!pserial_cfg) {
		ESP_LOGE(TAG, "Unexpected. No pserial_cfg found");
		return;
	}

	while (xQueueReceive(pserial_cfg->req_queue, &arg, portMAX_DELAY) == pdTRUE) {
		if (arg.type == PROTO_EVT_ENDPOINT) {
			/* Events */
			ESP_HEXLOGV("pserial_evt_rx", arg.data, arg.len, 32);
			ret = protocomm_pserial_ctrl_evnt_handler(pc, arg.epname, arg.data, arg.len, arg.msg_id);
		} else if (arg.type == PROTO_REQ_ENDPOINT) {
			/* Request */
			len = pserial_cfg->recv(arg.data, arg.len);
			if (len) {
				/*ESP_LOG_BUFFER_HEXDUMP("serial_rx", arg.data, len<16?len:16, ESP_LOG_INFO);*/
				ret = protocomm_pserial_ctrl_req_handler(pc, arg.data, len);
			}
		} else {
			ESP_LOGE(TAG, "Unexpected. Invalid type found in the packet");
			ret = ESP_FAIL;
		}

		if (ret)
			ESP_LOGI(TAG, "protobuf ctrl msg[0x%x] handling err[%d]", arg.msg_id, ret);

		if (arg.data) {
			free(arg.data);
			arg.data = NULL;
		}
	}

	ESP_LOGI(TAG, "Unexpected termination of pserial task");
}

static esp_err_t protocomm_pserial_start(protocomm_t *pc,
		pserial_xmit xmit, pserial_recv recv)
{
	struct pserial_config *pserial_cfg = NULL;

	if (pc == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	pc->add_endpoint = protocomm_pserial_add_ep;
	pc->remove_endpoint = protocomm_pserial_remove_ep;

	pserial_cfg = (struct pserial_config *) malloc(sizeof(struct pserial_config));
	if (pserial_cfg == NULL) {
		ESP_LOGE(TAG,"%s Failed to allocate memory", __func__);
		return ESP_ERR_NO_MEM;
	}
	pserial_cfg->xmit = xmit;
	pserial_cfg->recv = recv;
	pserial_cfg->req_queue = xQueueCreate(REQ_Q_MAX, sizeof(serial_arg_t));

	pc->priv = pserial_cfg;

	xTaskCreate(pserial_task, "pserial_task", CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE,
			(void *) pc, CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL);

	return ESP_OK;
}

static esp_err_t protocomm_pserial_stop(protocomm_t *pc)
{
	struct pserial_config *pserial_cfg = NULL;
	serial_arg_t arg = {0};

	// Validate input parameter
	if (!pc) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!pc->priv) {
		return ESP_OK; // Already stopped
	}

	pserial_cfg = (struct pserial_config *) pc->priv;

	// Clean up any remaining items in queue to prevent memory leaks
	while (xQueueReceive(pserial_cfg->req_queue, &arg, 0) == pdTRUE) {
		if (arg.data) {
			free(arg.data);
			arg.data = NULL;
		}
	}

	// Delete the queue - this will cause pserial_task to exit
	vQueueDelete(pserial_cfg->req_queue);

	// Give task a moment to notice queue deletion and exit
	vTaskDelay(pdMS_TO_TICKS(100));

	// Clean up configuration
	free(pserial_cfg);
	pc->priv = NULL;

	// Clear endpoint handlers
	pc->add_endpoint = NULL;
	pc->remove_endpoint = NULL;

	return ESP_OK;
}



/* ──────────────────────────────────────────────────────────────────────────
 * Registry-dispatch glue endpoint handlers
 *
 * "RPCReq": dispatches to registry request entries via msg_id extracted from
 *           proto field 2 (lightweight wire scanner, no full decode).
 * "RPCEvt": push-only; stub that rejects unexpected incoming requests.
 * ────────────────────────────────────────────────────────────────────────── */

static esp_err_t rpc_ll_slist_req_handler(uint32_t session_id,
                                          const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen,
                                          void *priv_data)
{
    (void)session_id; (void)priv_data;
    if (!inbuf || inlen <= 0 || !outbuf || !outlen) {
        ESP_LOGE(TAG, "slist_req_handler: invalid args");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t msg_id = 0;
    uint32_t field  = eh_proto_extract_msg_id(inbuf, (uint16_t)inlen, &msg_id);
    if (field != 2) {
        ESP_LOGE(TAG, "slist_req_handler: failed to extract msg_id (field=%"PRIu32")", field);
        return ESP_FAIL;
    }

    /* ── PENDING-007: Version-gated dispatch ──────────────────────────────
     * rpc_ver_negotiated is set during startup-event handshake (core.c).
     *
     * V1 namespace:  FG Linux  101–128   (CtrlMsgId)
     *                MCU       0x101–0x183 (RpcId)
     * V2 namespace:  Unified   0x400–0x5FF Req / 0x600–0x7FF Resp / 0x800–0x9FF Evt
     *                (RpcIdV2)
     *
     * Cross-version requests are rejected with ESP_ERR_NOT_SUPPORTED to
     * prevent routing V1 IDs to V2 handlers or vice-versa.
     * ────────────────────────────────────────────────────────────────────── */
    uint8_t ver = rpc_ver_negotiated;  /* single volatile read */

#define RPC_V2_REQ_BASE   0x400u
#define RPC_V2_REQ_MAX    0x5FFu
#define RPC_V2_RESP_BASE  0x600u
#define RPC_V2_RESP_MAX   0x7FFu
#define RPC_V2_EVT_BASE   0x800u
#define RPC_V2_EVT_MAX    0x9FFu
    bool is_v2_id = (msg_id >= RPC_V2_REQ_BASE && msg_id <= RPC_V2_EVT_MAX);

    if (ver == ESP_HOSTED_RPC_VERSION_V2) {
        if (!is_v2_id) {
            ESP_LOGW(TAG, "slist_req_handler: V2 negotiated but got V1 msg_id=0x%04"PRIx32", reject",
                     msg_id);
            *outbuf = NULL;
            *outlen = 0;
            return ESP_ERR_NOT_SUPPORTED;
        }
        ESP_LOGD(TAG, "slist_req_handler: V2 dispatch msg_id=0x%04"PRIx32" inlen=%d",
                 msg_id, (int)inlen);
    } else {
        /* V1 path — reject any V2-range msg_id */
        if (is_v2_id) {
            ESP_LOGW(TAG, "slist_req_handler: V1 negotiated but got V2 msg_id=0x%04"PRIx32", reject",
                     msg_id);
            *outbuf = NULL;
            *outlen = 0;
            return ESP_ERR_NOT_SUPPORTED;
        }
        ESP_LOGD(TAG, "slist_req_handler: V1 dispatch msg_id=0x%04"PRIx32" inlen=%d",
                 msg_id, (int)inlen);
    }
#undef RPC_V2_REQ_BASE
#undef RPC_V2_REQ_MAX
#undef RPC_V2_RESP_BASE
#undef RPC_V2_RESP_MAX
#undef RPC_V2_EVT_BASE
#undef RPC_V2_EVT_MAX

    /* ── Common dispatch via registry (version-agnostic; routes by msg_id range) */

    uint8_t *resp = NULL;
    uint16_t resp_len = 0;
    esp_err_t r = eh_cp_rpc_dispatch_req(msg_id,
                                                 inbuf, (uint16_t)inlen,
                                                 &resp, &resp_len);
    if (r != ESP_OK || !resp || resp_len == 0) {
        ESP_LOGW(TAG, "slist_req_handler: dispatch failed for 0x%04"PRIx32": %s",
                 msg_id, esp_err_to_name(r));
        if (resp) free(resp);
        *outbuf = NULL;
        *outlen = 0;
        return (r != ESP_OK) ? r : ESP_FAIL;
    }

    *outbuf = resp;
    *outlen = (ssize_t)resp_len;
    return ESP_OK;
}

/*
 * rpc_ll_slist_evt_handler — pass-through for pre-encoded event bytes.
 *
 * Design: In the registry event model, serialisation happens BEFORE this handler
 * is called (in registries.c send_event() → n->serialise()).  The encoded
 * proto bytes arrive in inbuf/inlen and must be returned unchanged so that
 * protocomm_pserial_ctrl_evnt_handler() can wrap them in a TLV and transmit.
 *
 * This is the correct pattern for CP→host events; there should never be an
 * incoming host request on this endpoint.  If one arrives, log and reject.
 * "Incoming" can be distinguished: inlen > 0 from the registry path (outgoing
 * event), vs. a real incoming request from the host which should not occur.
 */
static esp_err_t rpc_ll_slist_evt_handler(uint32_t session_id,
                                          const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen,
                                          void *priv_data)
{
    (void)session_id; (void)priv_data;

    if (!inbuf || inlen <= 0) {
        ESP_LOGE(TAG, "slist_evt_handler: unexpected empty request on event endpoint");
        *outbuf = NULL;
        *outlen = 0;
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Pre-encoded event: copy and pass through so compose_tlv + xmit can send. */
    uint8_t *buf = calloc(1, (size_t)inlen);
    if (!buf) {
        *outbuf = NULL;
        *outlen = 0;
        return ESP_ERR_NO_MEM;
    }
    memcpy(buf, inbuf, (size_t)inlen);
    *outbuf = buf;
    *outlen = inlen;
    return ESP_OK;
}

/* ────────────────────────────────────────────────────────────────────────── */

esp_err_t eh_cp_protocomm_init(protocomm_write_ll_cb_t write_cb, protocomm_read_ll_cb_t read_cb)
{
    if (g_protocomm_initialized) {
        ESP_LOGW(TAG, "Protocomm already initialized");
        return ESP_OK;
    }

    if (!write_cb || !read_cb) {
        ESP_LOGE(TAG, "Invalid transport callbacks: NULL");
        return ESP_ERR_INVALID_ARG;
    }

    g_write_cb = write_cb;
    g_read_cb  = read_cb;

    g_protocomm_instance = protocomm_new();
    if (!g_protocomm_instance) {
        ESP_LOGE(TAG, "Failed to allocate protocomm instance");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = protocomm_pserial_start(g_protocomm_instance, g_write_cb, g_read_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "protocomm_pserial_start failed: %s", esp_err_to_name(ret));
        protocomm_delete(g_protocomm_instance);
        g_protocomm_instance = NULL;
        return ret;
    }

    /* ── Register the two core registry-dispatch endpoints ────────────────── */
    ret = protocomm_add_endpoint(g_protocomm_instance,
                                 g_rpc_req_ep,
                                 rpc_ll_slist_req_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add RPCReq endpoint: %s", esp_err_to_name(ret));
        protocomm_pserial_stop(g_protocomm_instance);
        protocomm_delete(g_protocomm_instance);
        g_protocomm_instance = NULL;
        return ret;
    }

    ret = protocomm_add_endpoint(g_protocomm_instance,
                                 g_rpc_evt_ep,
                                 rpc_ll_slist_evt_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add RPCEvt endpoint: %s", esp_err_to_name(ret));
        protocomm_remove_endpoint(g_protocomm_instance, g_rpc_req_ep);
        protocomm_pserial_stop(g_protocomm_instance);
        protocomm_delete(g_protocomm_instance);
        g_protocomm_instance = NULL;
        return ret;
    }

    /* ── Endpoint registry mutex for legacy add_endpoint compat ──────────── */
    if (!g_endpoint_mutex) {
        g_endpoint_mutex = xSemaphoreCreateMutex();
        if (!g_endpoint_mutex) {
            ESP_LOGE(TAG, "Failed to create endpoint registry mutex");
            protocomm_remove_endpoint(g_protocomm_instance, g_rpc_req_ep);
            protocomm_remove_endpoint(g_protocomm_instance, g_rpc_evt_ep);
            protocomm_pserial_stop(g_protocomm_instance);
            protocomm_delete(g_protocomm_instance);
            g_protocomm_instance = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    g_protocomm_initialized = true;
    ESP_LOGI(TAG, "Protocomm initialized with registry dispatch endpoints");
    return ESP_OK;
}

esp_err_t eh_cp_protocomm_deinit(void)
{
    if (!g_protocomm_initialized) {
        ESP_LOGW(TAG, "%s Protocomm not initialized", __func__);
        return ESP_OK;
    }

    /* Clean up any remaining dynamic endpoints */
    if (g_endpoint_mutex) {
        xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);
        for (size_t i = 0; i < g_endpoint_count; i++) {
            ESP_LOGW(TAG, "Auto-cleaning up endpoint: %s", g_endpoint_list[i].name);
            free(g_endpoint_list[i].name);
        }
        free(g_endpoint_list);
        g_endpoint_list = NULL;
        g_endpoint_count = 0;
        g_endpoint_cap = 0;
        xSemaphoreGive(g_endpoint_mutex);
        vSemaphoreDelete(g_endpoint_mutex);
        g_endpoint_mutex = NULL;
    }

    protocomm_pserial_stop(g_protocomm_instance);
    protocomm_delete(g_protocomm_instance);

    g_protocomm_instance = NULL;
    g_write_cb = NULL;
    g_read_cb = NULL;
    g_protocomm_initialized = false;

    ESP_LOGI(TAG, "Protocomm deinitialized");
    return ESP_OK;
}

esp_err_t eh_cp_protocomm_process_rpc_evt(const char* epname, int event_id, void *data, int size)
{
    if (!g_protocomm_initialized || !g_protocomm_instance) {
        ESP_LOGE(TAG, "%s Protocomm not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if (unlikely(size < 0 || (size > 0 && !data))) {
        ESP_LOGE(TAG, "Invalid size: %d or data is NULL", size);
        return ESP_ERR_INVALID_ARG;
    }

    if (unlikely(size > MAX_SERIAL_DATA_SIZE)) {
        ESP_LOGE(TAG, "Data size too large: %d > %d", size, MAX_SERIAL_DATA_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    return protocomm_rpc_evt_ind(g_protocomm_instance, epname, data, size, event_id);
}

esp_err_t eh_cp_protocomm_process_rpc_req(uint8_t *data, int len)
{
    if (!g_protocomm_initialized || !g_protocomm_instance) {
        ESP_LOGE(TAG, "%s Protocomm not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if (unlikely(len < 0 || (len > 0 && !data))) {
        ESP_LOGE(TAG, "Invalid length: %d or data is NULL", len);
        return ESP_ERR_INVALID_ARG;
    }

    if (unlikely(len > MAX_SERIAL_DATA_SIZE)) {
        ESP_LOGE(TAG, "Data length too large: %d > %d", len, MAX_SERIAL_DATA_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    return protocomm_rpc_req_ind(g_protocomm_instance, data, len, PROTO_INVALID_RPC_MSG_ID);
}

/* Dynamic endpoint management API */
protocomm_t *eh_cp_protocomm_get_instance(void)
{
    if (!g_protocomm_initialized || !g_protocomm_instance) {
        ESP_LOGD(TAG, "%s Protocomm not initialized", __func__);
        return NULL;
    }
    return g_protocomm_instance;
}

#if EH_CP_LEGACY_ADD_ENDPOINT_API
/*
 * Legacy direct protocomm endpoint registration.
 * New extensions must use eh_cp_rpc_req_register() instead.
 * Controlled by CONFIG_ESP_HOSTED_LEGACY_ADD_ENDPOINT_API.
 */
esp_err_t eh_cp_protocomm_add_endpoint(const eh_cp_rpc_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "Invalid config: NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!config->endpoint_name || !config->handler) {
        ESP_LOGE(TAG, "Invalid parameters: endpoint_name=%p, handler=%p",
                 config->endpoint_name, config->handler);
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(config->endpoint_name) < MIN_EP_NAME_LEN || strlen(config->endpoint_name) > MAX_EP_NAME_LEN) {
        ESP_LOGE(TAG, "Invalid endpoint name length: %d (min=%d, max=%d)",
                 (int)strlen(config->endpoint_name), MIN_EP_NAME_LEN, MAX_EP_NAME_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_protocomm_instance) {
        ESP_LOGE(TAG, "%s Protocomm not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_endpoint_mutex) {
        ESP_LOGE(TAG, "Endpoint registry not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;

    xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);

    /* Check if endpoint already exists */
    for (size_t i = 0; i < g_endpoint_count; i++) {
        if (strcmp(g_endpoint_list[i].name, config->endpoint_name) == 0) {
            ESP_LOGE(TAG, "Endpoint already exists: %s", config->endpoint_name);
            ret = ESP_ERR_INVALID_STATE;
            goto unlock;
        }
    }

    /* Add to protocomm instance */
    ret = protocomm_add_endpoint(g_protocomm_instance, config->endpoint_name, config->handler, config->priv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add endpoint to protocomm: %s", esp_err_to_name(ret));
        goto unlock;
    }

    if (ensure_endpoint_capacity(g_endpoint_count + 1) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to grow endpoint registry");
        protocomm_remove_endpoint(g_protocomm_instance, config->endpoint_name);
        ret = ESP_ERR_NO_MEM;
        goto unlock;
    }

    g_endpoint_list[g_endpoint_count].name = strdup(config->endpoint_name);
    if (!g_endpoint_list[g_endpoint_count].name) {
        ESP_LOGE(TAG, "Failed to allocate memory for endpoint name");
        protocomm_remove_endpoint(g_protocomm_instance, config->endpoint_name);
        ret = ESP_ERR_NO_MEM;
        goto unlock;
    }
    g_endpoint_list[g_endpoint_count].handler = config->handler;
    g_endpoint_list[g_endpoint_count].priv_data = config->priv_data;
    g_endpoint_count++;

    ESP_LOGI(TAG, "Endpoint added successfully: %s", config->endpoint_name);

unlock:
    xSemaphoreGive(g_endpoint_mutex);
    return ret;
}
#endif /* EH_CP_LEGACY_ADD_ENDPOINT_API */

#if EH_CP_LEGACY_ADD_ENDPOINT_API
esp_err_t eh_cp_protocomm_remove_endpoint(const char *endpoint_name)
{
    if (!endpoint_name) {
        ESP_LOGE(TAG, "Invalid endpoint name: NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_protocomm_initialized || !g_protocomm_instance) {
        ESP_LOGE(TAG, "%s Protocomm not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if (!g_endpoint_mutex) {
        ESP_LOGE(TAG, "Endpoint registry not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_ERR_NOT_FOUND;

    xSemaphoreTake(g_endpoint_mutex, portMAX_DELAY);

    /* Find endpoint in registry */
    for (size_t i = 0; i < g_endpoint_count; i++) {
        if (strcmp(g_endpoint_list[i].name, endpoint_name) == 0) {
            /* Remove from protocomm instance */
            esp_err_t proto_ret = protocomm_remove_endpoint(g_protocomm_instance, endpoint_name);
            if (proto_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to remove endpoint from protocomm: %s", esp_err_to_name(proto_ret));
                ret = proto_ret;
                goto unlock;
            }

            /* Remove from registry */
            free(g_endpoint_list[i].name);
            if (i < g_endpoint_count - 1) {
                memmove(&g_endpoint_list[i], &g_endpoint_list[i + 1],
                        (g_endpoint_count - i - 1) * sizeof(*g_endpoint_list));
            }
            g_endpoint_count--;

            ESP_LOGI(TAG, "Endpoint removed successfully: %s", endpoint_name);
            ret = ESP_OK;
            break;
        }
    }

    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Endpoint not found: %s", endpoint_name);
    }

unlock:
    xSemaphoreGive(g_endpoint_mutex);
    return ret;
}
#endif /* EH_CP_LEGACY_ADD_ENDPOINT_API */
#endif /* EH_CP_FEAT_RPC_READY */
