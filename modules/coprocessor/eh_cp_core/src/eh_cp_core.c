/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "sys/queue.h"
#include "soc/soc.h"
#include "eh_cp_master_config.h"
#include <unistd.h>
#include <inttypes.h>
#ifndef CONFIG_IDF_TARGET_ARCH_RISCV
#include "xtensa/core-macros.h"
#endif
#include "esp_check.h"
#include "eh_header.h"
#include "eh_interface.h"
#include "esp_private/wifi.h"
#include "eh_transport_cp.h"

#include "driver/gpio.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "endian.h"
#include "eh_cp.h"
#include "eh_cp_event.h"
#include "eh_cp_rpc.h"
#include "eh_cp_utils.h"
#include "eh_common_fw_version.h"
#include "eh_cp_transport_test.h"
#include "eh_cp_core.h"        /* three-registry APIs */
#if EH_CP_FEAT_BT_READY
#include "eh_cp_feat_bt_core.h"
#endif
#include "eh_caps.h"             /* capability bit definitions */
#include "eh_common_tlv.h"              /* TLV type codes for PRIV handshake */
#include "eh_frame.h"            /* eh_frame_init for V2 upgrade */

//TODO: Fix this
/* Forward declarations */
static void auto_feat_init_task(void *pvParameters);        /* defined at bottom of this file */

#if EH_CP_FEAT_HOST_PS_READY
#include "eh_cp_feat_host_ps_apis.h"
#endif

#define BYPASS_TX_PRIORITY_Q 1
static const char TAG[] = "ehcp_core";


#define TO_HOST_QUEUE_SIZE               10

#define ETH_DATA_LEN                     1500
#define MAX_WIFI_STA_TX_RETRY            2

/* RX reassembly state for serial frames (assembled in core, dispatched to
 * Registry 1).  Uses heap allocation so there is no fixed size cap — the
 * buffer grows with each arriving fragment via realloc and is freed after
 * the complete message is dispatched. */
static struct serial_rx_data {
	uint16_t cur_seq_no;
	int      len;
	uint8_t *data;   /* heap-allocated; NULL when idle */
} s_serial_rx;

// Helper macro to get transport-specific GPIO reset pin
#if CONFIG_EH_TRANSPORT_CP_SPI
    #define ESP_GPIO_SLAVE_RESET CONFIG_EH_TRANSPORT_CP_SPI_GPIO_RESET
#elif CONFIG_EH_TRANSPORT_CP_SDIO
    #define ESP_GPIO_SLAVE_RESET CONFIG_EH_TRANSPORT_CP_SDIO_GPIO_RESET
#elif CONFIG_EH_TRANSPORT_CP_SPI_HD
    #define ESP_GPIO_SLAVE_RESET CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_RESET
#elif CONFIG_EH_TRANSPORT_CP_UART
    #define ESP_GPIO_SLAVE_RESET CONFIG_EH_TRANSPORT_CP_UART_GPIO_RESET
#else
    #define ESP_GPIO_SLAVE_RESET -1
#endif



volatile uint8_t datapath = 0;
volatile uint8_t station_connected = 0;
volatile uint8_t softap_started = 0;


/**
 * hdr_ver_negotiated: wire header version agreed with the host.
 * ESP_HOSTED_HDR_VERSION_V1 = not yet negotiated / V1 fallback (12-byte hdr)
 * ESP_HOSTED_HDR_VERSION_V2 = V2 header (20 bytes, magic 0xE9)
 *
 * Initialised to V1 so that PENDING-005 TX path code can safely read this
 * before the first ACK arrives (safe degraded mode, not undefined behaviour).
 * Updated by host_to_slave_reconfig() when ESP_PRIV_HEADER_VERSION_ACK parsed.
 */
volatile uint8_t hdr_ver_negotiated = ESP_HOSTED_HDR_VERSION_V1;

/*
 * rpc_ver_negotiated: agreed RPC protocol version.
 * ESP_HOSTED_RPC_VERSION_V1 = V1 (variant-specific proto, FG or MCU namespace)
 * ESP_HOSTED_RPC_VERSION_V2 = V2 (unified 0x400-0x9FF namespace)
 *
 * Initialised to V1 so dispatch code (PENDING-006) can read it safely before
 * the first handshake completes.
 * Updated by host_to_slave_reconfig() when ESP_PRIV_RPC_VERSION_ACK parsed.
 */
volatile uint8_t rpc_ver_negotiated = ESP_HOSTED_RPC_VERSION_V1;

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;
slave_config_t slv_cfg_g;
slave_state_t  slv_state_g;

#if !BYPASS_TX_PRIORITY_Q
static QueueHandle_t meta_to_host_queue = NULL;
static QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES] = {NULL};
#endif

esp_netif_t *slave_sta_netif = NULL;

SemaphoreHandle_t host_reset_sem;

static bool g_eh_cp_initialized = false;
static SemaphoreHandle_t g_init_mutex = NULL;

/* D2 — extension auto-init event group */
EventGroupHandle_t g_auto_feat_init_done_eg = NULL;


static void print_firmware_version(void);
static void populate_core_caps(void);   /* Registers built-in transport caps into accumulator */
esp_err_t eh_cp_init_internal(void);

#if EH_CP_FEAT_WIFI_READY
static inline esp_err_t populate_buff_handle(interface_buffer_handle_t *buf_handle,
		uint8_t if_type,
		uint8_t *buf,
		uint16_t len,
		void (*free_buf_func)(void *data),
		void *free_buf_handle,
		uint8_t flags,
		uint8_t if_num,
		uint16_t seq_num)
{
	buf_handle->if_type = if_type;
	buf_handle->payload = buf;
	buf_handle->payload_len = len;
	buf_handle->priv_buffer_handle = free_buf_handle;
	buf_handle->free_buf_handle = free_buf_func;
	buf_handle->flags = flags;
	buf_handle->if_num = if_num;
	buf_handle->seq_num = seq_num;

	return ESP_OK;
}

#define populate_wifi_buffer_handle(Buf_hdL, TypE, BuF, LeN) \
	populate_buff_handle(Buf_hdL, TypE, BuF, LeN, esp_wifi_internal_free_rx_buffer, eb, 0, 0, 0);


esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}
	ESP_HEXLOGV("AP_Get", buffer, len, 32);

	populate_wifi_buffer_handle(&buf_handle, ESP_AP_IF, buffer, len);

	if (send_to_host_queue(&buf_handle, PRIO_Q_OTHERS))
		goto DONE;

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

/* Default WLAN station RX callback implementation */
static esp_err_t default_wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void *data) = NULL;
	uint8_t *free_buff = NULL;

	if (eb) {
		free_func = esp_wifi_internal_free_rx_buffer;
		free_buff = (uint8_t *)eb;
	} else {
		free_func = free;
		free_buff = (uint8_t *)buffer;
	}

	ESP_LOGD(TAG, "default wlan_sta_rx_callback frame with len %u", len);

	if (!buffer || !datapath) {
		if (free_func && free_buff) {
			ESP_LOGD(TAG, "drop wifi packet. datapath: %u", datapath);
			free_func(free_buff);
		}
		return ESP_OK;
	}

	ESP_HEXLOGV("STA_Get", buffer, len, 64);

#if ESP_PKT_STATS
	pkt_stats.sta_lwip_in++;
#endif

	/* Send to Host */
	ESP_LOGD(TAG, "host packet");
	populate_buff_handle(&buf_handle, ESP_STA_IF, buffer, len, free_func, free_buff, 0, 0, 0);

	if (unlikely(send_to_host_queue(&buf_handle, PRIO_Q_OTHERS))) {
		if (free_func && free_buff) {
			free_func(free_buff);
		}
		return ESP_OK;
	}

#if ESP_PKT_STATS
	pkt_stats.sta_sh_in++;
	pkt_stats.sta_host_lwip_out++;
#endif

	return ESP_OK;
}
#endif

/* RX callback registry indexed by eh_if_type_t */
static eh_cp_rx_cb_t s_rx_cb[ESP_MAX_IF];

static void process_tx_pkt(interface_buffer_handle_t *buf_handle)
{
	int host_awake = 1;

	/* Check if data path is not yet open */
	if (!datapath) {
		/* Post processing */
		if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
			buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
			buf_handle->priv_buffer_handle = NULL;
		}
		ESP_LOGI(TAG, "wait for data path to be opened");
		vTaskDelay(pdMS_TO_TICKS(100));
		return;
	}

	if (if_context && if_context->if_ops && if_context->if_ops->write) {
        #if EH_CP_FEAT_HOST_PS_READY
        if (eh_cp_feat_host_ps_is_host_wakeup_needed(buf_handle)) {
            uint16_t wakeup_pkt_display_len = 32;
            ESP_LOGI(TAG, "Host sleeping, trigger wake-up");
        #if EH_CP_FEAT_HOST_PS_PRINT_FULL_WAKEUP_PACKET
            wakeup_pkt_display_len = buf_handle->payload_len>1600?1600:buf_handle->payload_len;
        #endif
            ESP_HEXLOGW("Wakeup_pkt", buf_handle->payload+H_ESP_PAYLOAD_HEADER_OFFSET,
                    buf_handle->payload_len, wakeup_pkt_display_len);
            host_awake = eh_cp_feat_host_ps_wakeup_host(portMAX_DELAY);
            buf_handle->flags |= FLAG_WAKEUP_PKT;
        }
        #endif

		if (host_awake)
			if_context->if_ops->write(if_handle, buf_handle);
		else
			ESP_LOGI(TAG, "Host wakeup failed, drop packet");
	}

	/* Post processing */
	if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
		buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
		buf_handle->priv_buffer_handle = NULL;
	}
}

/* Legacy eh_send_event_to_host() moved to eh_cp_feat_rpc. */

static esp_err_t host_to_slave_reconfig(uint8_t *evt_buf, uint16_t len)
{
	uint8_t len_left = len, tag_len;
	uint8_t *pos;
	int ret = ESP_OK;

	if (!evt_buf)
		return ESP_FAIL;

	pos = evt_buf;
	ESP_LOGD(TAG, "Init event length: %u", len);

	while (len_left) {
		tag_len = *(pos + 1);

		/* Bounds check: a well-formed TLV is type(1) + len(1) + value(tag_len). */
		if ((uint16_t)tag_len + 2 > len_left) {
			ESP_LOGW(TAG, "TLV truncated: tag=0x%02x len=%u remaining=%u — aborting parse",
			         *pos, tag_len, len_left);
			break;
		}

		if (*pos == HOST_CAPABILITIES) {

			ESP_LOGI(TAG, "Host capabilities: %2x", *pos);

		} else if (*pos == RCVD_ESP_FIRMWARE_CHIP_ID) {

			if (CONFIG_IDF_FIRMWARE_CHIP_ID != *(pos+2)) {
				ESP_LOGE(TAG, "Chip id returned[%u] doesn't match with chip id sent[%u]",
						*(pos+2), CONFIG_IDF_FIRMWARE_CHIP_ID);
			}

		} else if (*pos == SLV_CONFIG_TEST_RAW_TP) {
#if TEST_RAW_TP
			switch (*(pos + 2)) {

			case ESP_TEST_RAW_TP__ESP_TO_HOST:
				ESP_LOGI(TAG, "Raw TP ESP --> Host");
				/* TODO */
			break;

			case ESP_TEST_RAW_TP__HOST_TO_ESP:
				ESP_LOGI(TAG, "Raw TP ESP <-- Host");
				/* TODO */
			break;

			case ESP_TEST_RAW_TP__BIDIRECTIONAL:
				ESP_LOGI(TAG, "Raw TP ESP <--> Host");
				/* TODO */
			break;

			default:
				ESP_LOGW(TAG, "Unsupported Raw TP config");
			}

			eh_cp_process_transport_test_caps(*(pos + 2));
#else
			if (*(pos + 2))
				ESP_LOGW(TAG, "Host requested raw throughput testing, but not enabled in slave");
#endif
		} else if (*pos == SLV_CONFIG_THROTTLE_HIGH_THRESHOLD) {

			slv_cfg_g.throttle_high_threshold = *(pos + 2);
			ESP_LOGI(TAG, "ESP<-Host wifi flow ctl start thres [%u%%]",
					slv_cfg_g.throttle_high_threshold);

			/* Warn if FreeRTOS tick is small */
			if ((slv_cfg_g.throttle_low_threshold > 0) &&
			    (CONFIG_FREERTOS_HZ < 1000)) {
				ESP_LOGW(TAG, "FreeRTOS tick[%d]<1000. Enabling flow control with lower FrerRTOS tick may result in lower peak data throughput", (int) CONFIG_FREERTOS_HZ);
			}

		} else if (*pos == SLV_CONFIG_THROTTLE_LOW_THRESHOLD) {

			slv_cfg_g.throttle_low_threshold = *(pos + 2);
			ESP_LOGI(TAG, "ESP<-Host wifi flow ctl clear thres [%u%%]",
					slv_cfg_g.throttle_low_threshold);

		} else if (*pos == ESP_PRIV_HEADER_VERSION_ACK) {
			/* Host has acknowledged the wire header version to use.
			 * Value 2 → upgrade to V2 (20-byte, magic 0xE9) for all future frames.
			 * Value 1 → stay on V1 (host is old/doesn't support V2).
			 * Any other value → treat as V1 for safety.
			 */
			uint8_t acked_ver = *(pos + 2);
			if (acked_ver == ESP_HOSTED_HDR_VERSION_V2) {
				hdr_ver_negotiated = ESP_HOSTED_HDR_VERSION_V2;
				ESP_LOGI(TAG, "Wire header V2 negotiated (magic=0xE9, 20-byte hdr)");

				/* CRITICAL: Upgrade frame component to V2 after negotiation.
				 * Without this, frame encode/decode stays on V1 (12-byte) while
				 * host expects V2 (20-byte), causing checksum mismatches. */
				eh_frame_cfg_t frame_cfg;
#if defined(EH_CP_HOST_TYPE_MCU)
				/* MCU host variant */
#if CONFIG_EH_TRANSPORT_CP_SDIO
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_MCU_SDIO_DEFAULT;
#elif CONFIG_EH_TRANSPORT_CP_SPI
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_MCU_SPI_DEFAULT;
#elif CONFIG_EH_TRANSPORT_CP_SPI_HD
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_MCU_SPI_HD_DEFAULT;
#elif CONFIG_EH_TRANSPORT_CP_UART
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_MCU_UART_DEFAULT;
#endif
#else /* Linux FG host variant */
#if CONFIG_EH_TRANSPORT_CP_SDIO
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_FG_LINUX_SDIO_DEFAULT;
#elif CONFIG_EH_TRANSPORT_CP_SPI
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_FG_LINUX_SPI_DEFAULT;
#elif CONFIG_EH_TRANSPORT_CP_SPI_HD
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_FG_LINUX_SPI_HD_DEFAULT;
#elif CONFIG_EH_TRANSPORT_CP_UART
				frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_FG_LINUX_UART_DEFAULT;
#endif
#endif
#ifdef CONFIG_ESP_HOSTED_COMMON_CHECKSUM_ENABLED
				frame_cfg.checksum_enabled = 1;
#else
				frame_cfg.checksum_enabled = 0;
#endif
				frame_cfg.hdr_version = ESP_HOSTED_HDR_VERSION_V2;
				esp_err_t ret = eh_frame_init(&frame_cfg);
				if (ret == ESP_OK) {
					ESP_LOGI(TAG, "Frame component upgraded to V2 (20-byte header)");
				} else {
					ESP_LOGE(TAG, "Failed to upgrade frame component to V2: %d", ret);
				}
			} else {
				hdr_ver_negotiated = ESP_HOSTED_HDR_VERSION_V1;
				ESP_LOGI(TAG, "Wire header V1 retained (host acked ver=%u)", acked_ver);
			}

		} else if (*pos == ESP_PRIV_RPC_VERSION_ACK) {
			/* Host has acknowledged the RPC protocol version to use.
			 * Value 2 → V2 unified proto (msg_id 0x400-0x9FF namespace).
			 * Value 1 → V1 variant-specific proto (FG or MCU namespace).
			 * Any other value → treat as V1 for safety.
			 */
			uint8_t acked_rpc_ver = *(pos + 2);
			if (acked_rpc_ver == ESP_HOSTED_RPC_VERSION_V2) {
				rpc_ver_negotiated = ESP_HOSTED_RPC_VERSION_V2;
				ESP_LOGI(TAG, "RPC V2 negotiated (unified proto, msg_id 0x400+)");
			} else {
				rpc_ver_negotiated = ESP_HOSTED_RPC_VERSION_V1;
				ESP_LOGI(TAG, "RPC V1 retained (host acked ver=%u, variant-specific proto)", acked_rpc_ver);
			}

		} else if (*pos == ESP_PRIV_RPC_EP_ACK) {
			if (tag_len >= 1 && *(pos + 2) == 1) {
				ESP_LOGI(TAG, "RPC endpoint TLVs acknowledged by host");
			}

		} else {

			ESP_LOGD(TAG, "Unsupported H->S config: %2x", *pos);

		}

		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	{
		eh_cp_rpc_ep_config_t ep_cfg = {
			.req_ep = RPC_EP_NAME_REQ,
			.evt_ep = RPC_EP_NAME_EVT,
		};
		esp_event_post(EH_CP_EVENT, EH_CP_EVT_PRIVATE_RPC_READY,
		               &ep_cfg, sizeof(ep_cfg), portMAX_DELAY);
	}

	/* Negotiation summary — visible at boot for easy debugging */
	ESP_LOGI(TAG, "Negotiation complete: wire_hdr=V%u (%s), rpc=V%u (%s)",
		hdr_ver_negotiated,
		(hdr_ver_negotiated == ESP_HOSTED_HDR_VERSION_V2) ? "20-byte 0xE9" : "12-byte legacy",
		rpc_ver_negotiated,
		(rpc_ver_negotiated == ESP_HOSTED_RPC_VERSION_V2) ? "unified 0x400+" : "variant-specific");

	return ret;
}

static void process_priv_pkt(uint8_t *payload, uint16_t payload_len)
{
	int ret = 0;
	struct esp_priv_event *event;

	if (!payload || !payload_len)
		return;

	event = (struct esp_priv_event *) payload;

	if (eh_is_priv_event_init(event->event_type)) {

		ESP_LOGI(TAG, "Slave init_config received from host");
		ESP_HEXLOGD("init_config", event->event_data, event->event_len, 32);

		ret = host_to_slave_reconfig(event->event_data, event->event_len);
		if (ret) {
			ESP_LOGE(TAG, "failed to init event\n\r");
		}
	} else {
		ESP_LOGW(TAG, "Drop unknown event\n\r");
	}
}

static void process_rx_pkt(interface_buffer_handle_t *buf_handle)
{
	uint8_t *payload = NULL;
	uint16_t payload_len = 0;

#if EH_CP_FEAT_WIFI_READY
	int retry_wifi_tx = MAX_WIFI_STA_TX_RETRY;
#endif

	/* Use decoded fields from interface_buffer_handle_t (set by transport via
	 * eh_frame_decode). Do NOT re-parse the raw wire header here. */
	payload     = buf_handle->payload;
	payload_len = buf_handle->payload_len;

	ESP_HEXLOGV("bus_RX", payload, payload_len, 16);
	ESP_LOGD(TAG, "process_rx_pkt: if_type=%u flags=0x%02x seq=%u payload_len=%u",
	         buf_handle->if_type, buf_handle->flags, buf_handle->seq_num, payload_len);

	if (buf_handle->if_type == ESP_PRIV_IF) {
		process_priv_pkt(payload, payload_len);
#if EH_CP_FEAT_WIFI_READY
	} else if (buf_handle->if_type == ESP_STA_IF) {
		/* Try registry dispatch first — allows extensions to intercept STA frames.
		 * Falls back to direct WiFi TX if no RX callback is registered
		 * and station is connected (PENDING-009 resolved). */
		esp_err_t reg_ret = eh_cp_dispatch_rx(ESP_STA_IF, payload, payload_len, NULL);
		if (reg_ret == ESP_ERR_NOT_FOUND && station_connected) {
			/* No extension registered — forward directly to WiFi driver */
			int ret = 0;
			do {
				ret = esp_wifi_internal_tx(WIFI_IF_STA, payload, payload_len);
				if (ret) {
					vTaskDelay(pdMS_TO_TICKS(1));
				}
				retry_wifi_tx--;
			} while (ret && retry_wifi_tx);
  #if ESP_PKT_STATS
			if (ret)
				pkt_stats.hs_bus_sta_fail++;
			else
				pkt_stats.hs_bus_sta_out++;
  #endif
		}
	} else if (buf_handle->if_type == ESP_AP_IF) {
		/* Try registry dispatch first; fall back to direct WiFi AP TX. */
		esp_err_t reg_ret = eh_cp_dispatch_rx(ESP_AP_IF, payload, payload_len, NULL);
		if (reg_ret == ESP_ERR_NOT_FOUND && softap_started) {
			esp_wifi_internal_tx(WIFI_IF_AP, payload, payload_len);
			ESP_HEXLOGV("AP_Put", payload, payload_len, 32);
		}
#endif
	} else if (buf_handle->if_type == ESP_SERIAL_IF) {
#if ESP_PKT_STATS
		pkt_stats.serial_rx++;
#endif
		if (!buf_handle) {
			ESP_LOGE(TAG, "serial_rx_pkt: NULL handle");
			goto done;
		}

		ESP_LOGD(TAG, "serial_rx_pkt: seq=%u flags=0x%02x payload_len=%u assembled=%d",
		         buf_handle->seq_num, buf_handle->flags, buf_handle->payload_len,
		         s_serial_rx.len);

		if (!s_serial_rx.len) {
			/* First fragment of a new message */
			s_serial_rx.cur_seq_no = buf_handle->seq_num;
		}

		if (buf_handle->seq_num != s_serial_rx.cur_seq_no) {
			/* Sequence number mismatch — flush and discard stale buffer */
			ESP_LOGW(TAG, "serial_rx_pkt: seq mismatch (got %u expected %u), flushing",
			         buf_handle->seq_num, s_serial_rx.cur_seq_no);
			if (s_serial_rx.len > 0 && s_serial_rx.data) {
				(void)eh_cp_dispatch_rx(ESP_SERIAL_IF,
				                                s_serial_rx.data,
				                                (uint16_t)s_serial_rx.len,
				                                NULL);
			}
			free(s_serial_rx.data);
			s_serial_rx.data = NULL;
			s_serial_rx.len = 0;
			s_serial_rx.cur_seq_no = 0;
			goto done;
		}

		/* Grow the heap buffer to fit the new fragment */
		uint8_t *new_buf = realloc(s_serial_rx.data, s_serial_rx.len + payload_len);
		if (!new_buf) {
			ESP_LOGE(TAG, "serial_rx_pkt: realloc failed (need %d bytes)",
			         s_serial_rx.len + payload_len);
			free(s_serial_rx.data);
			s_serial_rx.data = NULL;
			s_serial_rx.len = 0;
			s_serial_rx.cur_seq_no = 0;
			goto done;
		}
		s_serial_rx.data = new_buf;
		memcpy(s_serial_rx.data + s_serial_rx.len, payload, payload_len);
		s_serial_rx.len += payload_len;

		if (!(buf_handle->flags & MORE_FRAGMENT)) {
			/* Last fragment — dispatch the fully reassembled message */
			if (s_serial_rx.len > UINT16_MAX) {
				ESP_LOGE(TAG, "serial_rx_pkt: reassembled msg too large (%d bytes), drop",
				         s_serial_rx.len);
			} else {
				(void)eh_cp_dispatch_rx(ESP_SERIAL_IF,
				                                s_serial_rx.data,
				                                (uint16_t)s_serial_rx.len,
				                                NULL);
			}
			/* dispatch_rx consumer (protocomm_pserial_data_ready) takes its
			 * own malloc copy — safe to free the reassembly buffer now */
			free(s_serial_rx.data);
			s_serial_rx.data = NULL;
			s_serial_rx.len = 0;
			s_serial_rx.cur_seq_no = 0;
		}

	}
	else if (buf_handle->if_type == ESP_HCI_IF) {
		(void)eh_cp_dispatch_rx(ESP_HCI_IF, payload, payload_len, NULL);
	}
#if TEST_RAW_TP
	else if (buf_handle->if_type == ESP_TEST_IF) {
		eh_transport_utils_update_raw_tp_rx_count(payload_len);
	}
#endif

	/* Free buffer handle */
done:
	if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
		buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
		buf_handle->priv_buffer_handle = NULL;
	}

}

/* Get data from host */
static void recv_task(void* pvParameters)
{
	interface_buffer_handle_t buf_handle = {0};

	for (;;) {

		if (!datapath) {
			/* Datapath is not enabled by host yet*/
			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		/* receive data from transport layer */
		if (if_context && if_context->if_ops && if_context->if_ops->read) {
			int len = if_context->if_ops->read(if_handle, &buf_handle);
			if (len <= 0) {
				vTaskDelay(pdMS_TO_TICKS(1));
				continue;
			}
			ESP_LOGD(TAG, "rx_read: len=%d if_type=%u flags=0x%02x seq=%u payload_len=%u",
			         len, buf_handle.if_type, buf_handle.flags, buf_handle.seq_num,
			         buf_handle.payload_len);
		}

		process_rx_pkt(&buf_handle);
	}
}


int send_to_host_queue(interface_buffer_handle_t *buf_handle, uint8_t queue_type)
{
#if BYPASS_TX_PRIORITY_Q
	process_tx_pkt(buf_handle);
	return ESP_OK;
#else
	int ret = xQueueSend(to_host_queue[queue_type], buf_handle, portMAX_DELAY);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Failed to send buffer into queue[%u]\n",queue_type);
		return ESP_FAIL;
	}

	if (queue_type == PRIO_Q_SERIAL)
		ret = xQueueSendToFront(meta_to_host_queue, &queue_type, portMAX_DELAY);
	else
		ret = xQueueSend(meta_to_host_queue, &queue_type, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Failed to send buffer into meta queue[%u]\n",queue_type);
		return ESP_FAIL;
	}

	return ESP_OK;
#endif
}

static void power_save_alert_task(void *pvParameters)
{
#if EH_CP_FEAT_HOST_PS_READY
    uint32_t event = (uint32_t)pvParameters;
	eh_cp_feat_host_ps_handle_alert(event);
#else
    (void)pvParameters;
#endif
	/* The task deletes itself after running. */
    vTaskDelete(NULL);
}

esp_err_t eh_cp_handle_power_save_alert(uint32_t event)
{
	return xTaskCreate(power_save_alert_task, "ps_alert_task", 3072, (void *)event, tskIDLE_PRIORITY + 5, NULL);
}

int event_handler(uint8_t val)
{
	switch(val) {
		case ESP_OPEN_DATA_PATH:
			ESP_EARLY_LOGI(TAG, "event_handler: ESP_OPEN_DATA_PATH");
			if (if_handle) {
				if_handle->state = ACTIVE;
				datapath = 1;
				ESP_EARLY_LOGI(TAG, "Open Data Path");
				if (host_reset_sem) {
					ESP_EARLY_LOGI(TAG, "Open Data Path: giving host_reset_sem");
					xSemaphoreGive(host_reset_sem);
				} else {
					ESP_EARLY_LOGI(TAG, "Failed to give host_reset_sem");
				}
			} else {
				ESP_EARLY_LOGI(TAG, "Failed to Start Data Path");
			}
			//eh_cp_handle_power_save_alert(ESP_OPEN_DATA_PATH);
			break;

		case ESP_CLOSE_DATA_PATH:
			datapath = 0;
			if (if_handle) {
				ESP_EARLY_LOGI(TAG, "Close Data Path");
				if (if_handle->state > DEACTIVE) {
					if_handle->state = DEACTIVE;
				}
			} else {
				ESP_EARLY_LOGI(TAG, "Failed to Stop Data Path");
			}
			break;

		case ESP_POWER_SAVE_ON:
			//host_power_save_alert(ESP_POWER_SAVE_ON);
			//if_handle->state = ACTIVE;
			eh_cp_handle_power_save_alert(ESP_POWER_SAVE_ON);
			break;

		case ESP_POWER_SAVE_OFF:
			ESP_EARLY_LOGI(TAG, "event_handler: ESP_POWER_SAVE_OFF");
			datapath = 1;
			if_handle->state = ACTIVE;
			//if (host_reset_sem) {
			//	xSemaphoreGive(host_reset_sem);
			//}
			//host_power_save_alert(ESP_POWER_SAVE_OFF);
			eh_cp_handle_power_save_alert(ESP_POWER_SAVE_OFF);
			break;
	}
	return 0;
}

#if defined(ESP_GPIO_SLAVE_RESET) && (ESP_GPIO_SLAVE_RESET != -1)
static void IRAM_ATTR gpio_resetpin_isr_handler(void* arg)
{

	ESP_EARLY_LOGI(TAG, "********* %p", if_handle);
	if (ESP_GPIO_SLAVE_RESET == -1) {
		ESP_EARLY_LOGI(TAG, "%s: using EN pin for slave reset", __func__);
		return;
	}

	static uint32_t lasthandshaketime_us;
	uint32_t currtime_us = esp_timer_get_time();

	if (gpio_get_level(ESP_GPIO_SLAVE_RESET) == 0) {
		lasthandshaketime_us = currtime_us;
	} else {
		uint32_t diff = currtime_us - lasthandshaketime_us;
		ESP_EARLY_LOGI(TAG, "%s Diff: %u", __func__, diff);
		if (diff < 500) {
			return; //ignore everything < half ms after an earlier irq
		} else {
			ESP_EARLY_LOGI(TAG, "Host triggered slave reset");
			esp_restart();
		}
	}
}

static void register_reset_pin(uint32_t gpio_num)
{
	if (gpio_num != -1) {
		ESP_LOGI(TAG, "Using GPIO [%lu] as slave reset pin", gpio_num);
		gpio_reset_pin(gpio_num);

		gpio_config_t slave_reset_pin_conf={
			.intr_type=GPIO_INTR_DISABLE,
			.mode=GPIO_MODE_INPUT,
			.pull_up_en=1,
			.pin_bit_mask=(1<<gpio_num)
		};

		gpio_config(&slave_reset_pin_conf);
		gpio_set_intr_type(gpio_num, GPIO_INTR_ANYEDGE);
		gpio_install_isr_service(0);
		gpio_isr_handler_add(gpio_num, gpio_resetpin_isr_handler, NULL);
	}
}
#endif

#if !BYPASS_TX_PRIORITY_Q
/* Send data to host */
static void send_task(void* pvParameters)
{
	uint8_t queue_type = 0;
	interface_buffer_handle_t buf_handle = {0};

	while (1) {

		if (!datapath) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		if (xQueueReceive(meta_to_host_queue, &queue_type, portMAX_DELAY))
			if (xQueueReceive(to_host_queue[queue_type], &buf_handle, portMAX_DELAY))
				process_tx_pkt(&buf_handle);
	}
}
#endif

static void host_reset_task(void* pvParameters)
{
	uint8_t capa = 0;
	uint32_t ext_capa = 0;
	uint8_t raw_tp_cap = 0;
	uint32_t feat_caps[EH_FEAT_CAPS_COUNT] = {0};

	ESP_LOGI(TAG, "host reset handler task started");

	while (1) {

		if (host_reset_sem) {
			xSemaphoreTake(host_reset_sem, portMAX_DELAY);
			ESP_LOGI(TAG, "host_reset_task: host_reset_sem taken, preparing slave-up TLV");
		} else {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		/* D8 — wait for all extensions to finish their init() before
		 * advertising capabilities. auto_feat_init_task sets this bit. */
		assert(g_auto_feat_init_done_eg); /* must be created before host_reset_task runs */
		xEventGroupWaitBits(g_auto_feat_init_done_eg, EH_CP_FEAT_INIT_DONE_BIT,
		                    pdFALSE, pdTRUE, portMAX_DELAY);
		capa = eh_cp_get_caps();
		ext_capa = eh_cp_get_ext_caps();
		raw_tp_cap = eh_cp_get_transport_test_raw_tp_conf();
		eh_cp_get_feat_caps(feat_caps);
		/* send capabilities (including feat_caps[8]) to host */
		ESP_LOGI(TAG,"Send slave up event");
		generate_startup_event(capa, ext_capa, raw_tp_cap, feat_caps);
	}
}


esp_err_t eh_cp_rx_register(eh_if_type_t if_type, eh_cp_rx_cb_t cb)
{
	if (if_type <= ESP_INVALID_IF || if_type >= ESP_MAX_IF) {
		return ESP_ERR_INVALID_ARG;
	}

	s_rx_cb[if_type] = cb;

#if EH_CP_FEAT_WIFI_READY
	if (if_type == ESP_STA_IF) {
		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, (wifi_rxcb_t)cb);
	} else if (if_type == ESP_AP_IF) {
		esp_wifi_internal_reg_rxcb(WIFI_IF_AP, (wifi_rxcb_t)cb);
	}
#endif

	return ESP_OK;
}

esp_err_t eh_cp_rx_unregister(eh_if_type_t if_type)
{
	return eh_cp_rx_register(if_type, NULL);
}

eh_cp_rx_cb_t eh_cp_rx_get(eh_if_type_t if_type)
{
	if (if_type <= ESP_INVALID_IF || if_type >= ESP_MAX_IF) {
		return NULL;
	}
	return s_rx_cb[if_type];
}

esp_err_t eh_cp_init_internal(void)
{
    if (g_eh_cp_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "ESP-Hosted core init");

    /* Core registries are zero-initialised; nothing to allocate here. */

    /* D2 — create the extension init-done event group */
    g_auto_feat_init_done_eg = xEventGroupCreate();
    if (!g_auto_feat_init_done_eg) {
        ESP_LOGE(TAG, "Failed to create ext_init_done event group");
        return ESP_ERR_NO_MEM;
    }
    /* Accumulate built-in transport capability bits */
    populate_core_caps();

    assert(host_reset_sem = xSemaphoreCreateBinary());
	xSemaphoreTake(host_reset_sem, 0);

    print_firmware_version();
#if EH_CP_FEAT_WIFI_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rx_register(ESP_STA_IF, default_wlan_sta_rx_callback));
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rx_register(ESP_AP_IF, wlan_ap_rx_callback));
#endif

#if defined(ESP_GPIO_SLAVE_RESET) && (ESP_GPIO_SLAVE_RESET != -1)
    register_reset_pin(ESP_GPIO_SLAVE_RESET);
#endif


//TODO: CONFIG_ESP_HOSTED_CP_AUTO_START_BT_ON_HOSTED_INIT
//also CONFIG_ESP_HOSTED_CP_AUTO_STOP_BT_ON_HOSTED_DEINIT need to support
#if 0 // Uncomment this if you wish to load bluetooth on slave startup always
#if EH_CP_FEAT_BT_READY && EH_CP_AUTO_START_BT
    eh_cp_bt_init();
	eh_cp_bt_enable();
#endif
#endif

    if_context = interface_insert_driver(event_handler);

#if CONFIG_EH_TRANSPORT_CP_SPI  /* transport Kconfig, not CP feature Kconfig — keep as-is */
    datapath = 1;
    if (host_reset_sem)
        xSemaphoreGive(host_reset_sem);
#endif

    if (!if_context || !if_context->if_ops) {
        ESP_LOGE(TAG, "Failed to insert driver\n");
        return ESP_FAIL;
    }

    if_handle = if_context->if_ops->init();

    if (!if_handle) {
        ESP_LOGE(TAG, "Failed to initialize driver\n");
        return ESP_FAIL;
    }

    assert(xTaskCreate(recv_task , "recv_task" ,
            EH_CP_TASK_STACK_SIZE, NULL ,
            EH_CP_TASK_PRIO_DEFAULT, NULL) == pdTRUE);

#if !BYPASS_TX_PRIORITY_Q
	meta_to_host_queue = xQueueCreate(TO_HOST_QUEUE_SIZE*3, sizeof(uint8_t));
	assert(meta_to_host_queue);
	for (uint8_t prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		to_host_queue[prio_q_idx] = xQueueCreate(TO_HOST_QUEUE_SIZE,
				sizeof(interface_buffer_handle_t));
		assert(to_host_queue[prio_q_idx]);
	}
	assert(xTaskCreate(send_task , "send_task" ,
			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_HOSTED_DEFAULT_TASK_PRIORITY, NULL) == pdTRUE);
#endif


    eh_cp_utils_create_debugging_tasks();
	eh_cp_create_transport_test_debugging_tasks();

    assert(xTaskCreate(host_reset_task, "host_reset_task" ,
            EH_CP_TASK_STACK_SIZE, NULL ,
            EH_CP_TASK_PRIO_DEFAULT, NULL) == pdTRUE);

    /* D2 — spawn feature auto-init task (if enabled) */
#if EH_CP_AUTO_FEAT_INIT
    assert(xTaskCreate(auto_feat_init_task, "auto_feat_init_task",
            EH_CP_TASK_STACK_SIZE, NULL,
            EH_CP_TASK_PRIO_DEFAULT, NULL) == pdTRUE);
#else
    /* Auto-init disabled — set done bit immediately so host_reset_task doesn't block */
    xEventGroupSetBits(g_auto_feat_init_done_eg, EH_CP_FEAT_INIT_DONE_BIT);
#endif

    g_eh_cp_initialized = true;
    ESP_LOGI(TAG, "ESP-Hosted core initialized");
    return ESP_OK;
}

esp_err_t eh_cp_init(void)
{
    // Public API - just calls internal init
    if (!g_init_mutex) {
        g_init_mutex = xSemaphoreCreateMutex();
    }

    /* Ensure the default event loop exists before auto_feat_init_task tries to
     * register handlers on EH_CP_EVENT.  If the app already created
     * it, this returns ESP_ERR_INVALID_STATE which we silently ignore. */
    esp_err_t loop_ret = esp_event_loop_create_default();
    if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
        return loop_ret;
    }

    xSemaphoreTake(g_init_mutex, portMAX_DELAY);
    esp_err_t ret = eh_cp_init_internal();
    xSemaphoreGive(g_init_mutex);
    return ret;
}

/* ===== Public RPC APIs ===== */
static void print_firmware_version(void)
{
	ESP_LOGI(TAG, "*********************************************************************");
	ESP_LOGI(TAG, "                ESP-Hosted Firmware version :: %s-%d.%d.%d",
			PROJECT_NAME, PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MINOR_1, PROJECT_VERSION_PATCH_1);
#if CONFIG_EH_TRANSPORT_CP_SPI
  #if EH_CP_BT_UART
	ESP_LOGI(TAG, "                Transport used :: SPI + UART                    ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SPI only                      ");
  #endif
#else
  #if EH_CP_BT_UART
	ESP_LOGI(TAG, "                Transport used :: SDIO + UART                   ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SDIO only                     ");
  #endif
#endif
	ESP_LOGI(TAG, "*********************************************************************");
}

/*
 * populate_core_caps() — accumulates built-in capability bits into the
 * Registry 2 accumulator (eh_cp_add_feat_cap_bits / eh_cp_add_feat_cap_bits_idx).
 *
 * This replaces the old get_capabilities() / get_capabilities_ext() that read
 * CONFIG_* macros directly.  Extensions contribute their own capability bits
 * by calling eh_cp_add_feat_cap_bits() from their own init() functions.
 * Core only adds the transport-level bits here.
 */
static void populate_core_caps(void)
{
    uint8_t  caps     = 0;
    uint32_t ext_caps = 0;

    ESP_LOGI(TAG, "Supported features are:");

#if CONFIG_EH_TRANSPORT_CP_SPI
    ESP_LOGI(TAG, "- WLAN over SPI");
    caps |= ESP_WLAN_SPI_SUPPORT;
#elif CONFIG_EH_TRANSPORT_CP_SDIO
    ESP_LOGI(TAG, "- WLAN over SDIO");
    caps |= ESP_WLAN_SDIO_SUPPORT;
#endif

#if CONFIG_EH_TRANSPORT_CP_SPI_CHECKSUM || \
    CONFIG_EH_TRANSPORT_CP_SDIO_CHECKSUM || \
    CONFIG_EH_TRANSPORT_CP_SPI_HD_CHECKSUM || \
    CONFIG_EH_TRANSPORT_CP_UART_CHECKSUM
    caps |= ESP_CHECKSUM_ENABLED;
#endif

#if EH_CP_FEAT_BT_READY
    /* BT capabilities are accumulated by the BT subsystem during its init.
     * Add a placeholder here; the actual bits come from eh_cp_bt_init(). */
    /* caps |= eh_cp_bt_get_capabilities();  -- called in bt_init instead */
#endif

    ESP_LOGI(TAG, "Supported extended features:");

#if CONFIG_EH_TRANSPORT_CP_SPI_HD
  #if (CONFIG_EH_TRANSPORT_CP_SPI_HD_NUM_DATA_LINES == 4)
    ESP_LOGI(TAG, "- SPI HD 4-bit interface");
    ext_caps |= EH_TRANSPORT_CP_SPI_HD_4_DATA_LINES;
  #elif (CONFIG_EH_TRANSPORT_CP_SPI_HD_NUM_DATA_LINES == 2)
    ESP_LOGI(TAG, "- SPI HD 2-bit interface");
    ext_caps |= EH_TRANSPORT_CP_SPI_HD_2_DATA_LINES;
  #else
    #error "Invalid SPI HD Number of Data Bits configuration"
  #endif
    ESP_LOGI(TAG, "- WLAN over SPI HD");
    ext_caps |= ESP_WLAN_SUPPORT;
#endif

#if CONFIG_EH_TRANSPORT_CP_UART
    ESP_LOGI(TAG, "- WLAN over UART");
    ext_caps |= ESP_WLAN_UART_SUPPORT;
#endif

    eh_cp_add_feat_cap_bits(caps, ext_caps);

    ESP_LOGI(TAG, "Core caps: 0x%02x  ext_caps: 0x%08"PRIx32, caps, ext_caps);
}

esp_err_t eh_cp_deinit(void)
{
    /* Call deinit on all registered extensions (D2) */
    for (const eh_cp_feat_desc_t *d = &_eh_cp_feat_descs_start;
         d < &_eh_cp_feat_descs_end; d++) {
        if (d->deinit_fn) {
            esp_err_t r = d->deinit_fn();
            if (r != ESP_OK) {
                ESP_LOGW(TAG, "ext '%s' deinit failed: %s",
                         d->name ? d->name : "?", esp_err_to_name(r));
            }
        }
    }
    ESP_LOGI(TAG, "ESP-Hosted CP deinitialized successfully");
    return ESP_OK;
}

/* ── D2: auto_feat_init_task — iterates linker-section descriptors —————————— */
static void auto_feat_init_task(void *pvParameters)
{
    size_t n = (size_t)(&_eh_cp_feat_descs_end - &_eh_cp_feat_descs_start);
    ESP_LOGI(TAG, "auto_feat_init_task: found %u extension descriptor(s)", (unsigned)n);

    if (n == 0) {
        ESP_LOGW(TAG, "auto_feat_init_task: no extensions registered via EH_CP_FEAT_REGISTER");
        xEventGroupSetBits(g_auto_feat_init_done_eg, EH_CP_FEAT_INIT_DONE_BIT);
        vTaskDelete(NULL);
        return;
    }

    /*
     * Sort descriptors by priority (ascending) using an insertion sort
     * over a stack-allocated pointer array.  n <= 16 typical so O(n^2) is fine.
     * We cannot sort the read-only linker section in place; sort pointer array.
     */
    const eh_cp_feat_desc_t *sorted[16]; /* hard cap at 16 extensions */
    if (n > 16) {
        ESP_LOGE(TAG, "auto_feat_init_task: too many extensions (%u > 16), truncating", (unsigned)n);
        n = 16;
    }
    for (size_t i = 0; i < n; i++) sorted[i] = &_eh_cp_feat_descs_start + i;
    /* insertion sort ascending by priority field */
    for (size_t i = 1; i < n; i++) {
        const eh_cp_feat_desc_t *key = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j-1]->priority > key->priority) {
            sorted[j] = sorted[j-1];
            j--;
        }
        sorted[j] = key;
    }

    for (size_t i = 0; i < n; i++) {
        const eh_cp_feat_desc_t *d = sorted[i];
		#if 0
        uintptr_t addr = (uintptr_t)d;
        if (addr < 0x3f000000 || addr > 0x44000000) {
            ESP_LOGE(TAG, "auto_feat_init_task: descriptor ptr %p out of IRAM/DRAM range, skipping", d);
            continue;
        }
		#endif
        if (!d->init_fn) {
            ESP_LOGW(TAG, "auto_feat_init_task: descriptor '%s' has NULL init_fn, skipping",
                     d->name ? d->name : "?");
            continue;
        }
        ESP_LOGI(TAG, "auto_feat_init_task: initialising '%s' (priority %d)",
                 d->name ? d->name : "?", d->priority);
        esp_err_t r = d->init_fn();
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "auto_feat_init_task: '%s' init() failed: %s",
                     d->name ? d->name : "?", esp_err_to_name(r));
            /* Non-fatal: continue with remaining extensions */
        }
    }

    ESP_LOGI(TAG, "auto_feat_init_task: all extensions initialised");
    xEventGroupSetBits(g_auto_feat_init_done_eg, EH_CP_FEAT_INIT_DONE_BIT);
    vTaskDelete(NULL);
}
