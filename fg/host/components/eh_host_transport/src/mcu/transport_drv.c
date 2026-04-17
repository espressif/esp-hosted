/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** Includes **/
#include <inttypes.h>

#include "esp_wifi.h"
#include "transport_drv.h"
#include "eh_transport.h"
#include "eh_host_transport_init.h"
#include "eh_host_transport_config.h"
#include "eh_host_fw_ver.h"
#include "stats.h"
#include "eh_log.h"
#include "serial_drv.h"
#include "serial_ll_if.h"
#include "mempool.h"
#include "stats.h"
#include "errno.h"
#include "hci_drv.h"
#include "port_eh_host_config.h"
#include "port_eh_host_log.h"
#include "eh_host_power_save.h"

#include "eh_host_cli.h"
#include "rpc_wrap.h"
#include "eh_host_transport_mcu_ops.h"

/**
 * @brief  Slave capabilities are parsed
 *         Currently no added functionality to that
 * @param  None
 * @retval None
 */

DEFINE_LOG_TAG(transport);
static char chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
void(*transport_eh_host_up_cb)(void) = NULL;
transport_channel_t *chan_arr[ESP_MAX_IF];
volatile uint8_t wifi_tx_throttling;
void *bus_handle = NULL;


static volatile uint8_t transport_state = TRANSPORT_INACTIVE;
/* Negotiated wire-header version: 1 = V1 (12B), 2 = V2 (20B). Default V1. */
volatile uint8_t host_hdr_ver_agreed = ESP_HOSTED_HDR_VERSION_V1;
/* Negotiated RPC protocol version: 1 = V1 (variant proto), 2 = V2 (unified 0x400+). Default V1. */
volatile uint8_t host_rpc_ver_agreed = ESP_HOSTED_RPC_VERSION_V1;

static void process_event(uint8_t *evt_buf, uint16_t len);
static int process_init_event(uint8_t *evt_buf, uint16_t len);

/* MCU transport ops functions */
extern esp_err_t eh_host_transport_mcu_ops_init(void);
extern void eh_host_transport_mcu_ops_deinit(void);


#if H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE && H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT != -1
static void *init_timeout_timer = NULL;

static void init_timeout_cb(void *arg)
{
	ESP_LOGE(TAG, "Init event not received within timeout, Reseting myself");
	g_h.funcs->_h_restart_host();
}
#endif

uint8_t is_transport_rx_ready(void)
{
	return (transport_state >= TRANSPORT_RX_ACTIVE);
}

uint8_t is_transport_tx_ready(void)
{
	return (transport_state >= TRANSPORT_TX_ACTIVE);
}

static void transport_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case TRANSPORT_TX_ACTIVE:
		{
			/* Initiate control path now */
			ESP_LOGI(TAG, "Base transport is set-up, TRANSPORT_TX_ACTIVE");
			if (transport_eh_host_up_cb)
				transport_eh_host_up_cb();
			transport_state = TRANSPORT_TX_ACTIVE;
			break;
		}

		case TRANSPORT_INACTIVE:
		case TRANSPORT_RX_ACTIVE:
			transport_state = event;
			break;

		default:
			break;
	}
}

void set_transport_state(uint8_t state)
{
	ESP_LOGI(TAG, "set_transport_state: %u", state);
	transport_driver_event_handler(state);
}

static void transport_drv_init(void)
{
	bus_handle = bus_init_internal();
	ESP_LOGD(TAG, "Bus handle: %p", bus_handle);
	assert(bus_handle);
#if H_NETWORK_SPLIT_ENABLED
	ESP_LOGI(TAG, "Network split enabled. Port ranges- Host:TCP(%d-%d), UDP(%d-%d), Slave:TCP(%d-%d), UDP(%d-%d)",
		H_HOST_TCP_LOCAL_PORT_RANGE_START, H_HOST_TCP_LOCAL_PORT_RANGE_END,
		H_HOST_UDP_LOCAL_PORT_RANGE_START, H_HOST_UDP_LOCAL_PORT_RANGE_END,
		H_SLAVE_TCP_REMOTE_PORT_RANGE_START, H_SLAVE_TCP_REMOTE_PORT_RANGE_END,
		H_SLAVE_UDP_REMOTE_PORT_RANGE_START, H_SLAVE_UDP_REMOTE_PORT_RANGE_END);
#endif
	hci_drv_init();
}

esp_err_t teardown_transport(void)
{
	#if H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE && H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT != -1
	/* Stop and cleanup init timeout timer if still active */
	if (init_timeout_timer) {
		g_h.funcs->_h_timer_stop(init_timeout_timer);
		init_timeout_timer = NULL;
	}
	#endif

	if (bus_handle) {
		bus_deinit_internal(bus_handle);
	}
	ESP_LOGI(TAG, "TRANSPORT_INACTIVE");
	transport_state = TRANSPORT_INACTIVE;
	return ESP_OK;
}

esp_err_t setup_transport(void(*eh_host_up_cb)(void))
{
	esp_err_t ret;

	/* Initialize MCU transport ops */
	ret = eh_host_transport_mcu_ops_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize MCU transport ops");
		return ret;
	}

	g_h.funcs->_h_hosted_init_hook();
	transport_drv_init();
	transport_eh_host_up_cb = eh_host_up_cb;

	return ESP_OK;
}

esp_err_t transport_drv_reconfigure(void)
{
	static int retry_slave_connection = 0;

	ESP_LOGI(TAG, "Attempt connection with slave: retry[%u]", retry_slave_connection);

#if H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE && H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT != -1
	/* Start init timeout timer if not already started */
	if (!init_timeout_timer) {
		init_timeout_timer = g_h.funcs->_h_timer_start("slave_unresponsive_timer", H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT, H_TIMER_TYPE_ONESHOT, init_timeout_cb, NULL);
		if (!init_timeout_timer) {
			ESP_LOGE(TAG, "Failed to create init timeout timer");
			return ESP_FAIL;
		}
		ESP_LOGI(TAG, "Started host communication init timer of %u seconds", H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT);
	}
#endif

	int retry_power_save_recover = 5;
	if (eh_host_woke_from_power_save()) {
		ESP_LOGI(TAG, "Waiting for power save to be off");
		g_h.funcs->_h_msleep(700);

		while (retry_power_save_recover) {
			if (is_transport_tx_ready()) {
				break;
			}
			retry_power_save_recover--;
		}
	}

	/* This would come into picture, only if the host has
	 * reset pin connected to slave's 'EN' or 'RST' GPIO */
	if (!is_transport_tx_ready()) {
		ensure_slave_bus_ready(bus_handle);
		transport_state = TRANSPORT_RX_ACTIVE;
		ESP_LOGI(TAG, "Waiting for esp_hosted slave to be ready");
		while (!is_transport_tx_ready()) {
			if (retry_slave_connection < MAX_RETRY_TRANSPORT_ACTIVE) {
				retry_slave_connection++;
				if (retry_slave_connection%50==0) {
					ESP_LOGI(TAG, "Not able to connect with ESP-Hosted slave device");
					ensure_slave_bus_ready(bus_handle);
				}
			} else {
				ESP_LOGW(TAG, "Failed to get ESP_Hosted slave transport up");
				return ESP_FAIL;
			}
			g_h.funcs->_h_msleep(200);
		}
	} else {
		ESP_LOGI(TAG, "Transport is already up");
	}

	retry_slave_connection = 0;
	return ESP_OK;
}

esp_err_t transport_drv_remove_channel(transport_channel_t *channel)
{
	if (!channel)
		return ESP_FAIL;

	switch (channel->if_type) {
	case ESP_AP_IF:
	case ESP_STA_IF:
		//Should we additionally do:
		//esp_wifi_internal_reg_rxcb(channel->if_type, NULL);
		break;
	case ESP_SERIAL_IF:
		/* TODO */
		break;
	default:
		break;
	}

	assert(chan_arr[channel->if_type] == channel);

	mempool_destroy(channel->memp);
	chan_arr[channel->if_type] = NULL;
	HOSTED_FREE(channel);

	return ESP_OK;
}

static void transport_sta_free_cb(void *buf)
{
	mempool_free(chan_arr[ESP_STA_IF]->memp, buf);
}

static void transport_ap_free_cb(void *buf)
{
	mempool_free(chan_arr[ESP_AP_IF]->memp, buf);
}

static void transport_serial_free_cb(void *buf)
{
	mempool_free(chan_arr[ESP_SERIAL_IF]->memp, buf);
}

static esp_err_t transport_drv_sta_tx(void *h, void *buffer, size_t len)
{
	void * copy_buff = NULL;

	if (!buffer || !len)
		return ESP_OK;

	if (unlikely(wifi_tx_throttling)) {
	#if ESP_PKT_STATS
		pkt_stats.sta_tx_flowctrl_drop++;
	#endif
		errno = -ENOBUFS;
		//return ESP_ERR_NO_BUFFS;
#if defined(ESP_ERR_ESP_NETIF_TX_FAILED)
		return ESP_ERR_ESP_NETIF_TX_FAILED;
#else
		return ESP_ERR_ESP_NETIF_NO_MEM;
#endif
	}

	assert(h && h==chan_arr[ESP_STA_IF]->api_chan);

	/*  Prepare transport buffer directly consumable */
	copy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_STA_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
	assert(copy_buff);
	g_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);

	return eh_host_transport_send_buffer(ESP_STA_IF, 0, copy_buff, len, H_BUFF_ZEROCOPY, copy_buff, transport_sta_free_cb, 0);
}

static esp_err_t transport_drv_ap_tx(void *h, void *buffer, size_t len)
{
	void * copy_buff = NULL;

	if (!buffer || !len)
		return ESP_OK;

	assert(h && h==chan_arr[ESP_AP_IF]->api_chan);

	/*  Prepare transport buffer directly consumable */
	copy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_AP_IF]->memp), MAX_TRANSPORT_BUFFER_SIZE, true);
	assert(copy_buff);
	g_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);

	return eh_host_transport_send_buffer(ESP_AP_IF, 0, copy_buff, len, H_BUFF_ZEROCOPY, copy_buff, transport_ap_free_cb, 0);
}

esp_err_t transport_drv_serial_tx(void *h, void *buffer, size_t len)
{
	/* TODO */
	assert(h && h==chan_arr[ESP_SERIAL_IF]->api_chan);
	return eh_host_transport_send_buffer(ESP_SERIAL_IF, 0, buffer, len, H_BUFF_NO_ZEROCOPY, buffer, transport_serial_free_cb, 0);
}


transport_channel_t *transport_drv_add_channel(void *api_chan,
		eh_if_type_t if_type, uint8_t secure,
		transport_channel_tx_fn_t *tx, const transport_channel_rx_fn_t rx)
{
	ESP_LOGD(TAG, "Adding channel IF[%u]: S[%u] Tx[%p] Rx[%p]", if_type, secure, tx, rx);
	transport_channel_t *channel = NULL;

	ESP_ERROR_CHECK(if_type >= ESP_MAX_IF);

	if (!tx || !rx) {
		ESP_LOGE(TAG, "%s fail for IF[%u]: tx or rx is NULL", __func__, if_type );
		return NULL;
	}

	if (chan_arr[if_type]) {
		/* Channel config already existed */
		ESP_LOGW(TAG, "Channel [%u] already created, replace with new callbacks", if_type);
		HOSTED_FREE(chan_arr[if_type]);
	}


	chan_arr[if_type] = g_h.funcs->_h_calloc(sizeof(transport_channel_t), 1);
	assert(chan_arr[if_type]);
	channel = chan_arr[if_type];

	switch (if_type) {

	case ESP_STA_IF:
		*tx = transport_drv_sta_tx;
		break;

	case ESP_AP_IF:
		*tx = transport_drv_ap_tx;
		break;

	case ESP_SERIAL_IF:
		*tx = transport_drv_serial_tx;
		break;

	default:
		//*tx = transport_drv_tx;
		ESP_LOGW(TAG, "Not yet suppported ESP_Hosted interface for if_type[%u]", if_type);
		return NULL;
	}

	channel->api_chan = api_chan;
	channel->if_type = if_type;
	channel->secure = secure;
	channel->tx = *tx;
	channel->rx = rx;

	/* Need to change size wrt transport */
	channel->memp = mempool_create(MAX_TRANSPORT_BUFFER_SIZE);
#ifdef H_USE_MEMPOOL
	assert(channel->memp);
#endif

	ESP_LOGI(TAG, "Add ESP-Hosted channel IF[%u]: S[%u] Tx[%p] Rx[%p]",
			if_type, secure, *tx, rx);

	return channel;
}

static void process_capabilities(uint8_t cap)
{
	ESP_LOGI(TAG, "capabilities: 0x%x",cap);
}

static uint32_t process_ext_capabilities(uint8_t * ptr)
{
	// ptr address may be not be 32-bit aligned
	uint32_t cap;

	cap = (uint32_t)ptr[0] +
		((uint32_t)ptr[1] << 8) +
		((uint32_t)ptr[2] << 16) +
		((uint32_t)ptr[3] << 24);
	ESP_LOGI(TAG, "extended capabilities: 0x%"PRIx32,cap);

	return cap;
}

void process_priv_communication(interface_buffer_handle_t *buf_handle)
{
	if (!buf_handle || !buf_handle->payload || !buf_handle->payload_len)
		return;

	process_event(buf_handle->payload, buf_handle->payload_len);
}

static void print_capabilities(uint32_t cap)
{
	ESP_LOGI(TAG, "Features supported are:");
	if (cap & ESP_WLAN_SDIO_SUPPORT)
		ESP_LOGI(TAG, "\t * WLAN");
	if (cap & ESP_BT_UART_SUPPORT)
		ESP_LOGI(TAG, "\t   - HCI over UART");
	if (cap & ESP_BT_SDIO_SUPPORT)
		ESP_LOGI(TAG, "\t   - HCI over SDIO");
	if (cap & ESP_BT_SPI_SUPPORT)
		ESP_LOGI(TAG, "\t   - HCI over SPI");
	if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
		ESP_LOGI(TAG, "\t   - BT/BLE dual mode");
	else if (cap & ESP_BLE_ONLY_SUPPORT)
		ESP_LOGI(TAG, "\t   - BLE only");
	else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
		ESP_LOGI(TAG, "\t   - BR EDR only");
}

static void print_ext_capabilities(uint8_t * ptr)
{
	// ptr address may be not be 32-bit aligned
	uint32_t cap;

	cap = (uint32_t)ptr[0] +
		((uint32_t)ptr[1] << 8) +
		((uint32_t)ptr[2] << 16) +
		((uint32_t)ptr[3] << 24);

	ESP_LOGI(TAG, "Extended Features supported:");
#if H_SPI_HD_HOST_INTERFACE
	if (cap & ESP_SPI_HD_INTERFACE_SUPPORT_2_DATA_LINES)
		ESP_LOGI(TAG, "\t * SPI HD 2 data lines interface");
	if (cap & ESP_SPI_HD_INTERFACE_SUPPORT_4_DATA_LINES)
		ESP_LOGI(TAG, "\t * SPI HD 4 data lines interface");
	if (cap & ESP_WLAN_SUPPORT)
		ESP_LOGI(TAG, "\t * WLAN");
	if (cap & ESP_BT_INTERFACE_SUPPORT)
		ESP_LOGI(TAG, "\t * BT/BLE");
#elif H_UART_HOST_TRANSPORT
	if (cap & ESP_WLAN_UART_SUPPORT)
		ESP_LOGI(TAG, "\t * WLAN over UART");
	if (cap & ESP_BT_VHCI_UART_SUPPORT)
		ESP_LOGI(TAG, "\t * BT over UART (VHCI)");
#else
	ESP_LOGI(TAG, "\t No extended features. capabilities[%" PRIu32 "]", cap);
#endif
}

static void process_event(uint8_t *evt_buf, uint16_t len)
{
	int ret = 0;
	struct esp_priv_event *event;

	if (!evt_buf || !len)
		return;

	event = (struct esp_priv_event *) evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {

		ESP_LOGI(TAG, "Received INIT event from ESP32 peripheral");
		ESP_HEXLOGD("Slave_init_evt", event->event_data, event->event_len, 32);

		ret = process_init_event(event->event_data, event->event_len);
		if (ret) {
			ESP_LOGE(TAG, "failed to init event\n\r");
		} else {

#if H_HOST_PS_ALLOWED && H_HOST_WAKEUP_GPIO
			eh_host_power_save_init();
#endif
		}
	} else {
		ESP_LOGW(TAG, "Drop unknown event\n\r");
	}
}

static esp_err_t get_chip_str_from_id(int chip_id, char* chip_str)
{
	int ret = ESP_OK;
	assert(chip_str);

	switch(chip_id) {
	case ESP_PRIV_FIRMWARE_CHIP_ESP32:
		strcpy(chip_str, "esp32");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C2:
		strcpy(chip_str, "esp32c2");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C3:
		strcpy(chip_str, "esp32c3");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C6:
		strcpy(chip_str, "esp32c6");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32S2:
		strcpy(chip_str, "esp32s2");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32S3:
		strcpy(chip_str, "esp32s3");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C5:
		strcpy(chip_str, "esp32c5");
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C61:
		strcpy(chip_str, "esp32c61");
		break;
	default:
		ESP_LOGW(TAG, "Unsupported chip id: %u", chip_id);
		strcpy(chip_str, "unsupported");
		ret = ESP_FAIL;
		break;
	}
	return ret;
}

static void verify_host_config_for_slave(uint8_t chip_type)
{
	uint8_t exp_chip_id = 0xff;


#if H_SLAVE_TARGET_ESP32
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32;
#elif H_SLAVE_TARGET_ESP32C2
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32C2;
#elif H_SLAVE_TARGET_ESP32C3
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32C3;
#elif H_SLAVE_TARGET_ESP32C6
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32C6;
#elif H_SLAVE_TARGET_ESP32S2
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32S2;
#elif H_SLAVE_TARGET_ESP32S3
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32S3;
#elif H_SLAVE_TARGET_ESP32C5
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32C5;
#elif H_SLAVE_TARGET_ESP32C61
	exp_chip_id = ESP_PRIV_FIRMWARE_CHIP_ESP32C61;
#else
	ESP_LOGW(TAG, "Incorrect host config for ESP slave chipset[%x]", chip_type);
#endif
	char slave_str[20] = {0};
	get_chip_str_from_id(chip_type, slave_str);

	if (chip_type!=exp_chip_id) {
		char exp_str[20] = {0};
		get_chip_str_from_id(exp_chip_id, exp_str);
		ESP_LOGE(TAG, "Identified slave [%s] != Expected [%s]\n\t\trun 'idf.py menuconfig' at host to reselect the slave?\n\t\tAborting.. ", slave_str, exp_str);
		g_h.funcs->_h_sleep(10);
		assert(0!=0);
	} else {
		ESP_LOGI(TAG, "Identified slave [%s]", slave_str);
		check_if_max_freq_used(chip_type);
	}
}

/** return values:
 * - 0 if versions as the same
 * - -1 if host version is smaller than slave version
 * - 1 if host version is bigger than slave version
 */
static int compare_fw_version(uint32_t slave_version)
{
	uint32_t host_version = EH_VERSION_VAL(EH_VERSION_MAJOR_1,
			EH_VERSION_MINOR_1,
			EH_VERSION_PATCH_1);

	// mask out patch level
	// compare major.minor only
	slave_version &= 0xFFFFFF00;
	host_version &= 0xFFFFFF00;

	if (host_version == slave_version) {
		// versions match
		return 0;
	} else if (host_version > slave_version) {
	    // host version > slave version
		ESP_LOGW(TAG, "=== ESP-Hosted Version Warning ===");
		printf("Version on Host is NEWER than version on co-processor\n");
		printf("RPC requests sent by host may encounter timeout errors\n");
		printf("or may not be supported by co-processor\n");
		ESP_LOGW(TAG, "=== ESP-Hosted Version Warning ===");
		return -1;
	} else {
	    // host version < slave version
		ESP_LOGW(TAG, "=== ESP-Hosted Version Warning ===");
		printf("Version on Host is OLDER than version on co-processor\n");
		printf("Host may not be compatible with co-processor\n");
		ESP_LOGW(TAG, "=== ESP-Hosted Version Warning ===");
		return 1;
	}
}

esp_err_t send_slave_config(uint8_t host_cap, uint8_t firmware_chip_id,
		uint8_t raw_tp_direction, uint8_t low_thr_thesh, uint8_t high_thr_thesh)
{
#define LENGTH_1_BYTE 1
	struct esp_priv_event *event = NULL;
	uint8_t *pos = NULL;
	uint16_t len = 0;
	uint8_t *sendbuf = NULL;

	sendbuf = g_h.funcs->_h_malloc_align(MEMPOOL_ALIGNED(256), MEMPOOL_ALIGNMENT_BYTES);
	assert(sendbuf);

	/* Populate event data */
	//event = (struct esp_priv_event *) (sendbuf + sizeof(struct esp_payload_header)); //ZeroCopy
	event = (struct esp_priv_event *) (sendbuf);

	event->event_type = ESP_PRIV_EVENT_INIT;

	/* Populate TLVs for event */
	pos = event->event_data;

	/* TLVs start */

	/* TLV - Board type */
	ESP_LOGI(TAG, "Slave chip Id[%x]", ESP_PRIV_FIRMWARE_CHIP_ID);
	*pos = HOST_CAPABILITIES;                          pos++;len++;
	*pos = LENGTH_1_BYTE;                              pos++;len++;
	*pos = host_cap;                                   pos++;len++;

	/* TLV - Capability */
	*pos = RCVD_ESP_FIRMWARE_CHIP_ID;                  pos++;len++;
	*pos = LENGTH_1_BYTE;                              pos++;len++;
	*pos = firmware_chip_id;                           pos++;len++;

	*pos = SLV_CONFIG_TEST_RAW_TP;                     pos++;len++;
	*pos = LENGTH_1_BYTE;                              pos++;len++;
	*pos = raw_tp_direction;                           pos++;len++;

	*pos = SLV_CONFIG_THROTTLE_HIGH_THRESHOLD;           pos++;len++;
	*pos = LENGTH_1_BYTE;                              pos++;len++;
	*pos = high_thr_thesh;                             pos++;len++;

	*pos = SLV_CONFIG_THROTTLE_LOW_THRESHOLD;           pos++;len++;
	*pos = LENGTH_1_BYTE;                              pos++;len++;
	*pos = low_thr_thesh;                              pos++;len++;

	ESP_LOGI(TAG, "raw_tp_dir[%s], flow_ctrl: low[%u] high[%u]",
			raw_tp_direction == ESP_TEST_RAW_TP__HOST_TO_ESP? "h2s":
			raw_tp_direction == ESP_TEST_RAW_TP__ESP_TO_HOST? "s2h":
			raw_tp_direction == ESP_TEST_RAW_TP__BIDIRECTIONAL? "bi-dir":
			"-", low_thr_thesh, high_thr_thesh);

	/* Acknowledge the negotiated wire-header version */
	*pos = ESP_PRIV_HEADER_VERSION_ACK;              pos++;len++;
	*pos = LENGTH_1_BYTE;                            pos++;len++;
	*pos = host_hdr_ver_agreed;                      pos++;len++;

	/* Acknowledge the negotiated RPC protocol version */
	*pos = ESP_PRIV_RPC_VERSION_ACK;                 pos++;len++;
	*pos = LENGTH_1_BYTE;                            pos++;len++;
	*pos = host_rpc_ver_agreed;                      pos++;len++;

	/* TLVs end */

	event->event_len = len;

	/* payload len = Event len + sizeof(event type) + sizeof(event len) */
	len += 2;

	return eh_host_transport_send_buffer(ESP_PRIV_IF, 0, sendbuf, len, H_BUFF_NO_ZEROCOPY, sendbuf, g_h.funcs->_h_free, 0);
}

static int transport_delayed_init(void)
{
	ESP_LOGI(TAG, "transport_delayed_init");
	rpc_start();
	/* Add up cli */
#ifdef H_ESP_HOSTED_CLI_ENABLED
	eh_host_cli_start();
#endif
	create_debugging_tasks();

	return 0;
}


static int process_init_event(uint8_t *evt_buf, uint16_t len)
{
	uint16_t len_left = len;
	uint8_t tag_len;
	uint8_t *pos;
	uint8_t raw_tp_config = H_TEST_RAW_TP_DIR;
	uint32_t ext_cap = 0;
	uint32_t slave_fw_version = 0;

	if (!evt_buf)
		return ESP_FAIL;

#if H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE && H_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT != -1
	/* Stop and delete the init timeout timer since we received the init event */
	if (init_timeout_timer) {
		g_h.funcs->_h_timer_stop(init_timeout_timer);
		init_timeout_timer = NULL;
		ESP_LOGI(TAG, "Init event received within timeout, cleared timer");
	}
#endif

	pos = evt_buf;
	ESP_LOGD(TAG, "Init event length: %u", len);

	while (len_left) {
		tag_len = *(pos + 1);

		/* Bounds check: malformed TLV must not cause len_left underflow. */
		if ((uint16_t)tag_len + 2 > len_left) {
			ESP_LOGW(TAG, "TLV truncated: tag=0x%02x len=%u remaining=%u — aborting",
			         *pos, tag_len, len_left);
			break;
		}

		if (*pos == ESP_PRIV_CAPABILITY) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			process_capabilities(*(pos + 2));
			print_capabilities(*(pos + 2));
		} else if (*pos == ESP_PRIV_CAP_EXT) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			ext_cap = process_ext_capabilities(pos + 2);
			print_ext_capabilities(pos + 2);
		} else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			chip_type = *(pos+2);
			verify_host_config_for_slave(chip_type);
		} else if (*pos == ESP_PRIV_TEST_RAW_TP) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
#if TEST_RAW_TP
			process_test_capabilities(*(pos + 2));
#else
			if (*(pos + 2))
				ESP_LOGW(TAG, "Slave enabled Raw Throughput Testing, but not enabled on Host");
#endif
		} else if (*pos == ESP_PRIV_RX_Q_SIZE) {
			ESP_LOGD(TAG, "slave rx queue size: %u", *(pos + 2));
		} else if (*pos == ESP_PRIV_TX_Q_SIZE) {
			ESP_LOGD(TAG, "slave tx queue size: %u", *(pos + 2));
		} else if (*pos == ESP_PRIV_FIRMWARE_VERSION) {
			// fw_version sent as a little-endian uint32_t
			slave_fw_version =
				*(pos + 2) |
				(*(pos + 3) << 8) |
				(*(pos + 4) << 16) |
				(*(pos + 5) << 24);
			ESP_LOGD(TAG, "slave fw version: 0x%08" PRIx32, slave_fw_version);
		} else if (*pos == ESP_PRIV_HEADER_VERSION) {
			/* Slave advertises its max supported wire-header version */
			uint8_t slave_hdr_ver = *(pos + 2);
			/* Agree on min(host_max, slave_proposed) — host supports V2 */
			host_hdr_ver_agreed = (slave_hdr_ver >= ESP_HOSTED_HDR_VERSION_V2)
				? ESP_HOSTED_HDR_VERSION_V2 : ESP_HOSTED_HDR_VERSION_V1;
			ESP_LOGI(TAG, "Wire hdr ver: slave=%u, agreed=%u",
				 slave_hdr_ver, host_hdr_ver_agreed);
		} else if (*pos == ESP_PRIV_RPC_VERSION) {
			/* Slave advertises its max supported RPC protocol version */
			uint8_t slave_rpc_ver = *(pos + 2);
			/* Agree on min(host_max, slave_proposed) — host supports V2 */
			host_rpc_ver_agreed = (slave_rpc_ver >= ESP_HOSTED_RPC_VERSION_V2)
				? ESP_HOSTED_RPC_VERSION_V2 : ESP_HOSTED_RPC_VERSION_V1;
			ESP_LOGI(TAG, "RPC ver: slave=%u, agreed=%u",
				 slave_rpc_ver, host_rpc_ver_agreed);
		} else {
			ESP_LOGD(TAG, "Unsupported EVENT: %2x", *pos);
		}
		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	// if ESP_PRIV_FIRMWARE_VERSION was not received, slave version will be 0.0.0
	compare_fw_version(slave_fw_version);

	if ((chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S2) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S3) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C2) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C3) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C6) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C5) &&
		(chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C61)) {
		ESP_LOGI(TAG, "ESP board type is not mentioned, ignoring [%d]\n\r", chip_type);
		chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
		return -1;
	} else {
		ESP_LOGI(TAG, "ESP board type is : %d \n\r", chip_type);
	}

	if (ext_cap) {
#if H_SPI_HD_HOST_INTERFACE
		// reconfigure SPI_HD interface based on host and slave capabilities
		if (H_SPI_HD_HOST_NUM_DATA_LINES == 4) {
			// SPI_HD on host is configured to use 4 data bits
			if (ext_cap & ESP_SPI_HD_INTERFACE_SUPPORT_4_DATA_LINES) {
				// slave configured to use 4 bits
				ESP_LOGI(TAG, "configure SPI_HD interface to use 4 data lines");
				g_h.funcs->_h_spi_hd_set_data_lines(H_SPI_HD_CONFIG_4_DATA_LINES);
			} else {
				// slave configured to use 2 bits
				ESP_LOGI(TAG, "configure SPI_HD interface to use 2 data lines");
				g_h.funcs->_h_spi_hd_set_data_lines(H_SPI_HD_CONFIG_2_DATA_LINES);
			}
		} else {
			// SPI_HD on host is configured to use 2 data bits
			if (ext_cap & ESP_SPI_HD_INTERFACE_SUPPORT_4_DATA_LINES) {
				// slave configured to use 4 bits
				ESP_LOGI(TAG, "SPI_HD on slave uses 4 data lines but Host is configure to use 2 data lines");
				g_h.funcs->_h_spi_hd_set_data_lines(H_SPI_HD_CONFIG_2_DATA_LINES);
			} else {
				// slave configured to use 2 bits
				ESP_LOGI(TAG, "configure SPI_HD interface to use 2 data lines");
				g_h.funcs->_h_spi_hd_set_data_lines(H_SPI_HD_CONFIG_2_DATA_LINES);
			}
		}
#endif
	}

	transport_driver_event_handler(TRANSPORT_TX_ACTIVE);

	ESP_ERROR_CHECK(send_slave_config(0, chip_type, raw_tp_config,
		H_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD,
		H_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD));

	transport_delayed_init();

	return 0;
}

int serial_rx_handler(interface_buffer_handle_t * buf_handle)
{
	return serial_ll_rx_handler(buf_handle);
}
