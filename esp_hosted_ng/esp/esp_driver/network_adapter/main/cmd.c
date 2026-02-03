/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "esp_log.h"
#include "interface.h"
#include "esp_log.h"
#include "esp.h"
#include "cmd.h"
#include "adapter.h"
#include "endian.h"
#include "esp_private/wifi.h"
#include "esp_wpa.h"
#include "app_main.h"
#include "esp_wifi.h"
#include "esp_wifi_driver.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_check.h"
#include "wifi_defs.h"
#include <time.h>
#include <sys/time.h>
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "freertos/event_groups.h"

#define TAG "FW_CMD"

static bool verify_ota = false;
static uint8_t broadcast_mac[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ETH_ALEN) == 0)

/* This is limitation of ESP WiFi lib.
 * It needs the password field updated to understand
 * we are triggering non-open connection */
#define DUMMY_PASSPHRASE            "12345678"

#define RESET_TIMEOUT             (5*1000)
extern volatile uint8_t station_connected;
extern volatile uint8_t association_ongoing;
extern volatile uint8_t softap_started;

volatile uint8_t sta_init_flag;

static struct wpa_funcs wpa_cb;
static esp_event_handler_instance_t instance_any_id;
static uint8_t *ap_bssid;

extern uint32_t ip_address;
extern struct macfilter_list mac_list;
extern uint8_t dev_mac[MAC_ADDR_LEN];

uint8_t dummy_mac[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
uint8_t dummy_mac2[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x66};

int esp_wifi_register_wpa_cb_internal(struct wpa_funcs *cb);
int esp_wifi_unregister_wpa_cb_internal(void);
extern esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);
extern volatile uint8_t power_save_on;
extern void wake_host();
extern struct wow_config wow;

#if defined(CONFIG_SOC_WIFI_HE_SUPPORT)
#define TX_DONE_PREFIX 8
#else
#define TX_DONE_PREFIX 0
#endif
static const esp_partition_t* update_partition = NULL;
static esp_ota_handle_t handle;

extern int wpa_parse_wpa_ie(const u8 *wpa_ie, size_t wpa_ie_len, wifi_wpa_ie_t *data);
static inline void WPA_PUT_LE16(u8 *a, u16 val)
{
    a[1] = val >> 8;
    a[0] = val & 0xff;
}

static void esp_wifi_set_debug_log()
{
    /* set WiFi log level and module */
    uint32_t g_wifi_log_module = WIFI_LOG_MODULE_WIFI;
    uint32_t g_wifi_log_submodule = 0;

    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_ALL;
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_INIT;
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_IOCTL;
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_CONN;
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_SCAN;

    esp_wifi_internal_set_log_mod(g_wifi_log_module, g_wifi_log_submodule, true);

    esp_wifi_internal_set_log_level(WIFI_LOG_VERBOSE);
}

static int send_command_resp(uint8_t if_type,
                             uint8_t cmd_code, uint8_t cmd_status,
                             uint8_t *data, uint32_t len, uint32_t offset)
{
    int ret;
    struct command_header *header;
    interface_buffer_handle_t buf_handle = {0};

    ESP_LOGD(TAG, "Sending response of cmd=%d status=%d\n", cmd_code, cmd_status);

    buf_handle.payload_len = sizeof(struct command_header) +
                             sizeof(struct esp_payload_header) +
                             len;
    buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
    if (!buf_handle.payload) {
        ESP_LOGE(TAG, "Malloc send buffer fail!");
        return ESP_FAIL;
    }
    memset(buf_handle.payload, 0, buf_handle.payload_len);

    if (data && len) {
        memcpy(buf_handle.payload + offset, data, len);
    }

    header = (struct command_header *) buf_handle.payload;

    header->cmd_status = cmd_status;
    header->cmd_code = cmd_code;
    header->len = len;

    buf_handle.if_type = if_type;
    buf_handle.if_num = 0;
    buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

    buf_handle.priv_buffer_handle = buf_handle.payload;
    buf_handle.free_buf_handle = free;

    /* Send command response */
    ret = send_command_response(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
        goto cleanup;
    }

    return ESP_OK;
cleanup:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }

    return ret;
}

static void deinitialize_wifi(void)
{
    /*esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
      &esp_scan_done_event_handler);*/
    esp_wifi_stop();
    esp_wifi_deinit();
}

static void cleanup_ap_bssid(void)
{
    ESP_LOGI(TAG, "%s", __func__);
    if (ap_bssid) {
        free(ap_bssid);
        ap_bssid = NULL;
    }
}

bool sta_init(void)
{
    return true;
}

bool sta_deinit(void)
{
    return true;
}

int sta_connection(uint8_t *bssid)
{
    if (!bssid) {
        return ESP_FAIL;
    }

    ap_bssid = (uint8_t*)malloc(MAC_ADDR_LEN);
    if (!ap_bssid) {
        ESP_LOGI(TAG, "%s:%u malloc failed\n", __func__, __LINE__);
        return ESP_FAIL;
    }
    memcpy(ap_bssid, bssid, MAC_ADDR_LEN);
    esp_wifi_sta_connect_internal(ap_bssid);

    return ESP_OK;
}

int station_rx_eapol(uint8_t *src_addr, uint8_t *buf, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    interface_buffer_handle_t buf_handle = {0};
    uint8_t * tx_buf = NULL;
    u8 own_mac[MAC_ADDR_LEN] = {0};

    if (!src_addr || !buf || !len) {
        ESP_LOGI(TAG, "eapol err - src_addr: %p buf: %p len: %lu\n",
                 src_addr, buf, len);
        //TODO : free buf using esp_wifi_internal_free_rx_buffer?
        return ESP_FAIL;
    }

    if (!ap_bssid) {
        ESP_LOGI(TAG, "AP bssid null\n");
        return ESP_FAIL;
    }

    ret = esp_wifi_get_macaddr_internal(0, own_mac);
    if (ret) {
        ESP_LOGI(TAG, "Failed to get own sta MAC addr\n");
        return ESP_FAIL;
    }

#if CONFIG_ESP_SDIO_HOST_INTERFACE
    if (power_save_on && wow.four_way_handshake) {
        ESP_LOGI(TAG, "Wakeup on FourWayHandshake");
        wake_host();
        buf_handle.flag = 0xFF;
        sleep(1);
    }
#endif

    /* Check destination address against self address */
    if (memcmp(ap_bssid, src_addr, MAC_ADDR_LEN)) {
        /* Check for multicast or broadcast address */
        //if (!(own_mac[0] & 1))
        ESP_LOG_BUFFER_HEXDUMP("src_addr", src_addr, MAC_ADDR_LEN, ESP_LOG_INFO);
        return ESP_FAIL;
    }

#if 0
    if (len) {
        ESP_LOG_BUFFER_HEXDUMP("RXEapol", buf, len, ESP_LOG_INFO);
    }
#endif

    tx_buf = (uint8_t *)malloc(len);
    if (!tx_buf) {
        ESP_LOGE(TAG, "%s:%u malloc failed\n", __func__, __LINE__);
        return ESP_FAIL;
    }

    memcpy((char *)tx_buf, buf, len);

    buf_handle.if_type = ESP_STA_IF;
    buf_handle.if_num = 0;
    buf_handle.payload_len = len;
    buf_handle.payload = tx_buf;
    buf_handle.wlan_buf_handle = tx_buf;
    buf_handle.free_buf_handle = free;
    buf_handle.pkt_type = PACKET_TYPE_EAPOL;

    ret = send_frame_to_host(&buf_handle);

    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send buffer\n");
        goto DONE;
    }

    return ESP_OK;

DONE:
    free(tx_buf);
    tx_buf = NULL;

    return ESP_FAIL;
}

static bool in_4way(void)
{
    ESP_LOGI(TAG, "STA in 4 way\n");
    return false;
}

static int sta_michael_mic_failure(uint16_t is_unicast)
{
    ESP_LOGI(TAG, "STA mic fail\n");
    return ESP_OK;
}

void disconnected_cb(uint8_t reason_code)
{
    ESP_LOGI(TAG, "STA disconnected [%u]\n", reason_code);
}

static int prepare_event(uint8_t if_type, interface_buffer_handle_t *buf_handle, uint16_t event_len)
{
    esp_err_t ret = ESP_OK;

    buf_handle->if_type = if_type;
    buf_handle->if_num = 0;
    buf_handle->payload_len = event_len;
    buf_handle->pkt_type = PACKET_TYPE_EVENT;

    buf_handle->payload = heap_caps_malloc(buf_handle->payload_len, MALLOC_CAP_DMA);
    if (!buf_handle->payload) {
        ESP_LOGE(TAG, "Failed to allocate event buffer\n");
        return ESP_FAIL;
    }
    memset(buf_handle->payload, 0, buf_handle->payload_len);

    buf_handle->priv_buffer_handle = buf_handle->payload;
    buf_handle->free_buf_handle = free;

    return ret;
}

void config_done(void)
{
}

void (wpa_sta_clear_current_pmksa)(void)
{
}

uint8_t *owe_build_dh_ie(uint16_t group)
{
    return NULL;
}

int process_owe_assoc_resp(const u8 *rsn_ie, size_t rsn_len, const uint8_t *dh_ie, size_t dh_len)
{
    return 0;
}

static void handle_scan_event(void)
{
    //uint32_t type = 0;
    interface_buffer_handle_t buf_handle = {0};
    struct event_header *header;
    esp_err_t ret = ESP_OK;

    /*type = ~(1 << WLAN_FC_STYPE_BEACON) & ~(1 << WLAN_FC_STYPE_PROBE_RESP);*/
    /*esp_wifi_register_mgmt_frame_internal(type, 0);*/

    ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct event_header));
    if (ret) {
        ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
        return;
    }

    header = (struct event_header *) buf_handle.payload;

    header->event_code = EVENT_SCAN_RESULT;
    header->len = 0;
    header->status = 0;

    ret = send_command_event(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send scan done event\n");
        goto DONE;
    }

    return;

DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }
    return;
}

void handle_sta_disconnected_event(wifi_event_sta_disconnected_t *disconnected, bool wakeup_flag)
{
    interface_buffer_handle_t buf_handle = {0};
    struct disconnect_event *event;
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "STA Disconnect event: %d\n", disconnected->reason);

    ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct disconnect_event));
    if (ret) {
        ESP_LOGE(TAG, "Failed to prepare event buffer\n");
        cleanup_ap_bssid();
        return;
    }

    event = (struct disconnect_event *) buf_handle.payload;

    event->header.event_code = EVENT_STA_DISCONNECT;
    event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
    event->header.status = 0;

    memcpy(event->ssid, disconnected->ssid, disconnected->ssid_len);
    memcpy(event->bssid, disconnected->bssid, MAC_ADDR_LEN);
    event->reason = disconnected->reason;

    if (wakeup_flag) {
        buf_handle.flag = 0xFF;
    }
    ret = send_command_event(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send disconnect event\n");
        goto DONE;
    }

    cleanup_ap_bssid();
    return;

DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }
    cleanup_ap_bssid();
    return;
}

static int sta_rx_assoc(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
                        uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
    interface_buffer_handle_t buf_handle = {0};
    struct assoc_event *connect;
    esp_err_t ret = ESP_OK;
    struct ieee_mgmt_header *ieee_mh;
    uint8_t mac[MAC_ADDR_LEN];

    ESP_LOGI(TAG, "STA connect event [channel %d]\n", channel);
    ESP_LOG_BUFFER_HEXDUMP("BSSID", sender, MAC_ADDR_LEN, ESP_LOG_INFO);

    ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct assoc_event)
                        + len + IEEE_HEADER_SIZE);
    if (ret) {
        ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
        return ESP_FAIL;
    }

    connect = (struct assoc_event *) buf_handle.payload;

    connect->header.event_code = EVENT_ASSOC_RX;
    connect->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
    connect->header.status = 0;

    /* Populate event body */
    connect->frame_type = type;
    connect->channel = channel;
    memcpy(connect->bssid, sender, MAC_ADDR_LEN);
    connect->rssi = htole32(rssi);
    connect->tsf = htole64(current_tsf);

    /* Add IEEE mgmt header */
    ieee_mh = (struct ieee_mgmt_header *) connect->frame;

    ieee_mh->frame_control = type << 4;

    ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(ieee_mh->da, mac, MAC_ADDR_LEN);
    memcpy(ieee_mh->sa, sender, MAC_ADDR_LEN);
    memcpy(ieee_mh->bssid, sender, MAC_ADDR_LEN);

    connect->frame_len = htole16(len + IEEE_HEADER_SIZE);
    memcpy(connect->frame + IEEE_HEADER_SIZE, frame, len);

    /*ESP_LOG_BUFFER_HEXDUMP(TAG, connect->frame, connect->frame_len, ESP_LOG_INFO);*/

    ret = send_command_event(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send assoc resp event\n");
        goto DONE;
    }

    return ESP_OK;

DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }
    return ret;
}

static IRAM_ATTR void esp_wifi_tx_done_cb(uint8_t ifidx, uint8_t *data,
                                          uint16_t *len, bool txstatus)
{
    if (ifidx == ESP_IF_WIFI_STA) {
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    esp_err_t result;
    bool wakeup_flag = false;

    if (event_base != WIFI_EVENT) {
        ESP_LOGI(TAG, "Received unregistered event %s[%lu]\n", event_base, event_id);
        return;
    }

    switch (event_id) {

    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "station started");
        sta_init_flag = 1;
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wifi Station Connected event!! \n");
        association_ongoing = 0;
        station_connected = 1;

        result = esp_wifi_set_tx_done_cb(esp_wifi_tx_done_cb);
        if (result) {
            ESP_LOGE(TAG, "Failed to set tx done cb\n");
        }

        result = esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t)wlan_sta_rx_callback);
        if (result) {
            ESP_LOGE(TAG, "Failed to set rx cb\n");
        }

        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        if (!event_data) {
            ESP_LOGE(TAG, "%s:%u NULL data", __func__, __LINE__);
            return;
        }
#if CONFIG_ESP_SDIO_HOST_INTERFACE
        if (power_save_on && wow.disconnect) {
            /* Wake-up host always on disconnect */
            ESP_LOGI(TAG, "Wakeup on disconnect");
            wake_host();
            wakeup_flag = true;
            sleep(1);
        }
#endif
        handle_sta_disconnected_event((wifi_event_sta_disconnected_t*) event_data, wakeup_flag);
        station_connected = 0;
        /*esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);*/
        break;

    case WIFI_EVENT_SCAN_DONE:
        ESP_LOGI(TAG, "wifi scanning done");
        handle_scan_event();
        break;

    case WIFI_EVENT_AP_START:
        ESP_LOGI(TAG, "softap started");
        softap_started = 1;
        break;

    case WIFI_EVENT_AP_STOP:
        ESP_LOGI(TAG, "softap stopped");
        softap_started = 0;
        break;

    case WIFI_EVENT_STA_STOP:
        ESP_LOGI(TAG, "Station stop");
        sta_init_flag = 0;
        break;

    default:
        ESP_LOGI(TAG, "Unregistered event: %lu\n", event_id);
    }
}

void esp_create_wifi_event_loop(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
}

static int sta_rx_auth(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
                       uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
    struct auth_event *event;
    esp_err_t ret = ESP_OK;
    interface_buffer_handle_t buf_handle = {0};
    struct ieee_mgmt_header *ieee_mh;
    uint8_t mac[MAC_ADDR_LEN];

    ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct auth_event)
                        + len + IEEE_HEADER_SIZE);
    if (ret) {
        ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
        return ESP_FAIL;
    }

    event = (struct auth_event *) buf_handle.payload;

    /* Populate event header */
    event->header.event_code = EVENT_AUTH_RX;
    event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
    /*event->header.status = 1;*/

    /* Populate event body */
    event->frame_type = type;
    event->channel = channel;
    memcpy(event->bssid, sender, MAC_ADDR_LEN);
    event->rssi = htole32(rssi);
    event->tsf = htole64(current_tsf);

    /* Add IEEE mgmt header */
    ieee_mh = (struct ieee_mgmt_header *) event->frame;

    ieee_mh->frame_control = type << 4;

    ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(ieee_mh->da, mac, MAC_ADDR_LEN);
    memcpy(ieee_mh->sa, sender, MAC_ADDR_LEN);
    memcpy(ieee_mh->bssid, sender, MAC_ADDR_LEN);

    event->frame_len = htole16(len + IEEE_HEADER_SIZE);
    memcpy(event->frame + IEEE_HEADER_SIZE, frame, len);

    /*ESP_LOG_BUFFER_HEXDUMP(TAG, event->frame, event->frame_len, ESP_LOG_INFO);*/

    ret = send_command_event(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send auth event\n");
        goto DONE;
    }

    return ESP_OK;

DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }
    return ret;
}

static int sta_rx_probe(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
                        uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
    struct scan_event *event;
    esp_err_t ret = ESP_OK;
    interface_buffer_handle_t buf_handle = {0};

    /*ESP_LOGI(TAG, "SCAN# Type: %d, Channel: %d, Len: %lu\n", type, channel, len);
    ESP_LOG_BUFFER_HEXDUMP("Frame", frame, len, ESP_LOG_INFO);
    ESP_LOG_BUFFER_HEXDUMP("MAC", sender, MAC_ADDR_LEN, ESP_LOG_INFO);
    */

    ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct scan_event) + len);
    if (ret) {
        ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
        return ESP_FAIL;
    }

    event = (struct scan_event *) buf_handle.payload;

    /* Populate event header */
    event->header.event_code = EVENT_SCAN_RESULT;
    event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
    event->header.status = 1;

    /* Populate event body */
    event->frame_type = type;
    event->channel = channel;
    memcpy(event->bssid, sender, MAC_ADDR_LEN);
    event->rssi = htole32(rssi);
    event->frame_len = htole16(len);
    event->tsf = htole64(current_tsf);
    memcpy(event->frame, frame, len);

    ret = send_command_event(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send scan event\n");
        goto DONE;
    }

    return ESP_OK;

DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }
    return ret;
}

static int is_valid_assoc_resp(uint8_t *frame, size_t len, uint8_t *src_addr)
{
    /*ESP_LOG_BUFFER_HEXDUMP("assoc src addr", src_addr, MAC_ADDR_LEN, ESP_LOG_INFO);*/

    if (!ap_bssid || !len) {
        ESP_LOGE(TAG, "%s:%u AP bssid is not found, Return failure\n",
                 __func__, __LINE__);
        return false;
    }

    if (!len || !frame) {
        ESP_LOGE(TAG, "%s:%u Invalid args, Return failure\n", __func__, __LINE__);
        return false;
    }

    if (memcmp(ap_bssid, src_addr, MAC_ADDR_LEN)) {
        ESP_LOGI(TAG, "%s:%u Assoc response from unexpected AP, failure\n",
                 __func__, __LINE__);
        return false;
    }

    /*ESP_LOG_BUFFER_HEXDUMP("assoc frame:", frame, len, ESP_LOG_INFO);*/

    if (frame[IE_POS_ASSOC_RESP_STATUS] == 0) {
        return true;
    }

    return false;
}

static int handle_wpa_sta_rx_mgmt(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
                                  uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
    if (!sender) {
        ESP_LOGI(TAG, "%s:%u src mac addr NULL", __func__, __LINE__);
        return ESP_FAIL;
    }

    switch (type) {

    case WLAN_FC_STYPE_BEACON:
        /*ESP_LOGV(TAG, "%s:%u beacon frames ignored\n", __func__, __LINE__);*/
        sta_rx_probe(type, frame, len, sender, rssi, channel, current_tsf);
        break;

    case WLAN_FC_STYPE_PROBE_RESP:
        /*ESP_LOGV(TAG, "%s:%u probe response\n", __func__, __LINE__);*/
        sta_rx_probe(type, frame, len, sender, rssi, channel, current_tsf);
        break;

    case WLAN_FC_STYPE_AUTH:
        ESP_LOGI(TAG, "%s:%u Auth[%u] recvd\n", __func__, __LINE__, type);
        /*ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, ESP_LOG_INFO);*/
        sta_rx_auth(type, frame, len, sender, rssi, channel, current_tsf);
        break;

    case WLAN_FC_STYPE_ASSOC_RESP:
    case WLAN_FC_STYPE_REASSOC_RESP:

        ESP_LOGI(TAG, "%s:%u ASSOC Resp[%u] recvd\n", __func__, __LINE__, type);

        if (is_valid_assoc_resp(frame, len, sender)) {
            /* In case of open authentication,
             * connected event denotes connection established.
             * Whereas in case of secured authentication
             * It just triggers m1-m4 4 way handshake.
             */
            sta_rx_assoc(type, frame, len, sender, rssi, channel, current_tsf);
        }
        break;

    default:
        ESP_LOGI(TAG, "%s:%u Unsupported type[%u], ignoring\n", __func__, __LINE__, type);

    }

    return ESP_OK;
}

static int hostap_sta_join(uint8_t *bssid, uint8_t *wpa_ie, uint8_t wpa_ie_len,
                           uint8_t* rsnxe, uint16_t rsnxe_len, bool *pmf_enable, int subtype, uint8_t *pairwise_cipher)
{
    return true;
}

static int wpa_ap_remove(uint8_t* bssid)
{
    return true;
}

static uint8_t  *wpa_ap_get_wpa_ie(uint8_t *ie_len)
{
    *ie_len = 0;
    return NULL;
}

static bool wpa_ap_rx_eapol(uint8_t *addr, uint8_t *buf, size_t len)
{
    esp_err_t ret = ESP_OK;
    interface_buffer_handle_t buf_handle = {0};
    uint8_t * tx_buf = NULL;

    if (!buf || !len) {
        ESP_LOGI(TAG, "eapol err - buf: %p len: %zu\n",
                 buf, len);
        //TODO : free buf using esp_wifi_internal_free_rx_buffer?
        return ESP_FAIL;
    }

    tx_buf = (uint8_t *)malloc(len);
    if (!tx_buf) {
        ESP_LOGE(TAG, "%s:%u malloc failed\n", __func__, __LINE__);
        return ESP_FAIL;
    }

    memcpy((char *)tx_buf, buf, len);

#if 0
    if (len) {
        ESP_LOG_BUFFER_HEXDUMP("tx_buf", tx_buf, len, ESP_LOG_INFO);
    }
#endif

    buf_handle.if_type = ESP_AP_IF;
    buf_handle.if_num = 0;
    buf_handle.payload_len = len;
    buf_handle.payload = tx_buf;
    buf_handle.wlan_buf_handle = tx_buf;
    buf_handle.free_buf_handle = free;
    buf_handle.pkt_type = PACKET_TYPE_EAPOL;

    ESP_LOGD(TAG, "Sending eapol to host on AP iface\n");
    ret = send_frame_to_host(&buf_handle);

    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send buffer\n");
        goto DONE;
    }

    return true;

DONE:
    free(tx_buf);
    tx_buf = NULL;

    return false;
}

static void wpa_ap_get_peer_spp_msg(void *sm_data, bool *spp_cap, bool *spp_req)
{
    return;
}

char hostapd;
static int *hostap_init(void)
{
    return NULL;
}

static int hostap_deinit(void *data)
{
    return true;
}

static int handle_wpa_ap_rx_mgmt(void *pkt, uint32_t pkt_len, uint8_t chan, int rssi, int nf)
{
    struct mgmt_event *event;
    esp_err_t ret = ESP_OK;
    interface_buffer_handle_t buf_handle = {0};
    pkt_len = pkt_len + 24;

    //ESP_LOG_BUFFER_HEXDUMP(TAG, pkt, pkt_len, ESP_LOG_INFO);
    ret = prepare_event(ESP_AP_IF, &buf_handle, sizeof(struct mgmt_event)
                        + pkt_len);
    if (ret) {
        ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
        return ESP_FAIL;
    }

    event = (struct mgmt_event *) buf_handle.payload;

    /* Populate event header */
    event->header.event_code = EVENT_AP_MGMT_RX;
    event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
    /*event->header.status = 1;*/

    memcpy(event->frame, pkt, pkt_len);
    event->frame_len = pkt_len;
    event->chan = chan;
    event->rssi = rssi;
    event->nf = nf;

#if 0
    if (event->frame[0] != 0x40) {
        ESP_LOGD(TAG, "%s: Got packet type as %x \n", __func__, event->frame[0]);
    }

    ESP_LOG_BUFFER_HEXDUMP(TAG, event->frame, event->frame_len, ESP_LOG_INFO);
#endif
    ret = send_command_event(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send mgmt frames\n");
        goto DONE;
    }

    return ESP_OK;

DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }
    return ret;

    return 0;
}

static void sta_connected(uint8_t *bssid)
{
}

extern char * wpa_config_parse_string(const char *value, size_t *len);
static void sta_disconnected(uint8_t reason_code)
{
}

esp_err_t initialise_wifi(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_err_t result = esp_wifi_init(&cfg);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Init internal failed");
        return result;
    }

    esp_wifi_set_debug_log();

    /* Register to get events from wifi driver */
    esp_create_wifi_event_loop();
    /* Register callback functions with wifi driver */
    memset(&wpa_cb, 0, sizeof(struct wpa_funcs));

    wpa_cb.wpa_sta_rx_mgmt = handle_wpa_sta_rx_mgmt;
    wpa_cb.wpa_sta_init = sta_init;
    wpa_cb.wpa_sta_deinit = sta_deinit;
    wpa_cb.wpa_sta_connect = sta_connection;
    wpa_cb.wpa_sta_connected_cb = sta_connected;
    wpa_cb.wpa_sta_disconnected_cb = sta_disconnected;
    wpa_cb.wpa_sta_disconnected_cb = disconnected_cb;
    wpa_cb.wpa_sta_rx_eapol = station_rx_eapol;
    wpa_cb.wpa_sta_in_4way_handshake = in_4way;
    wpa_cb.wpa_parse_wpa_ie = wpa_parse_wpa_ie;
    wpa_cb.wpa_michael_mic_failure = sta_michael_mic_failure;
    wpa_cb.wpa_config_done = config_done;
    wpa_cb.wpa_config_parse_string  = wpa_config_parse_string;
    wpa_cb.wpa_sta_clear_curr_pmksa = wpa_sta_clear_current_pmksa;
    wpa_cb.owe_build_dhie = owe_build_dh_ie;
    wpa_cb.owe_process_assoc_resp = process_owe_assoc_resp;

    wpa_cb.wpa_ap_join       = hostap_sta_join;
    wpa_cb.wpa_ap_remove     = wpa_ap_remove;
    wpa_cb.wpa_ap_get_wpa_ie = wpa_ap_get_wpa_ie;
    wpa_cb.wpa_ap_rx_eapol   = wpa_ap_rx_eapol;
    wpa_cb.wpa_ap_get_peer_spp_msg  = wpa_ap_get_peer_spp_msg;
    wpa_cb.wpa_ap_init       = hostap_init;
    wpa_cb.wpa_ap_deinit     = hostap_deinit;
    wpa_cb.wpa_ap_rx_mgmt    = handle_wpa_ap_rx_mgmt;

    esp_wifi_register_wpa_cb_internal(&wpa_cb);

    result = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (result) {
        ESP_LOGE(TAG, "Failed to set wifi mode\n");
    }

    return result;
}

int process_start_scan(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    uint32_t type = 0;
    wifi_scan_config_t params = {0};
    esp_err_t ret = ESP_OK;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
    struct scan_request *scan_req;
    bool config_present = false;

    /* Register to receive probe response and beacon frames */
    type = (1 << WLAN_FC_STYPE_BEACON) | (1 << WLAN_FC_STYPE_PROBE_RESP) |
           (1 << WLAN_FC_STYPE_ASSOC_RESP) | (1 << WLAN_FC_STYPE_REASSOC_RESP);
    esp_wifi_register_mgmt_frame_internal(type, 0);

    scan_req = (struct scan_request *) payload;

    if (strlen(scan_req->ssid)) {
        params.ssid = malloc(sizeof(scan_req->ssid));
        assert(params.ssid);

        memcpy(params.ssid, scan_req->ssid, sizeof(scan_req->ssid));
        params.scan_type = 0;
        config_present = true;
    }

    if (scan_req->channel) {
        params.channel = scan_req->channel;
        config_present = true;
    }

    if (sta_init_flag || softap_started) {
        /* Trigger scan */
        if (config_present) {
            ret = esp_wifi_scan_start(&params, false);
        } else {
            ret = esp_wifi_scan_start(NULL, false);
        }

        if (ret) {
            ESP_LOGI(TAG, "Scan failed ret=[0x%x]\n", ret);
            cmd_status = CMD_RESPONSE_FAIL;

            /* Reset frame registration */
            esp_wifi_register_mgmt_frame_internal(0, 0);
        }
    } else {
        ESP_LOGI(TAG, "Scan not permitted as WiFi is not yet up");
        cmd_status = CMD_RESPONSE_FAIL;

        /* Reset frame registration */
        esp_wifi_register_mgmt_frame_internal(0, 0);
    }

    if (params.ssid) {
        free(params.ssid);
    }

    ret = send_command_resp(if_type, CMD_SCAN_REQUEST, cmd_status, NULL, 0, 0);

    return ret;
}

int process_set_mcast_mac_list(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_set_mcast_mac_addr *cmd_mcast_mac_list;

    cmd_mcast_mac_list = (struct cmd_set_mcast_mac_addr *) payload;

    mac_list.count = cmd_mcast_mac_list->count;
    memcpy(mac_list.mac_addr, cmd_mcast_mac_list->mcast_addr,
           sizeof(mac_list.mac_addr));

    /*ESP_LOG_BUFFER_HEXDUMP("MAC Filter", (uint8_t *) &mac_list, sizeof(mac_list), ESP_LOG_INFO);*/

    ret = send_command_resp(if_type, CMD_SET_MCAST_MAC_ADDR, CMD_RESPONSE_SUCCESS, NULL, 0, 0);

    return ret;
}

int process_tx_power(uint8_t if_type, uint8_t *payload, uint16_t payload_len, uint8_t cmd)
{
    esp_err_t ret = ESP_OK;
    struct cmd_set_get_val *val;
    int8_t max_tx_power;
    uint32_t value;

    if (cmd == CMD_SET_TXPOWER) {
        val = (struct cmd_set_get_val *)payload;
        max_tx_power = val->value;
        esp_wifi_set_max_tx_power(max_tx_power);
    }

    esp_wifi_get_max_tx_power((int8_t *)&max_tx_power);
    value = max_tx_power;
    ret = send_command_resp(if_type, cmd, CMD_RESPONSE_SUCCESS, (uint8_t *)&value,
                            sizeof(uint32_t), sizeof(struct command_header));

    return ret;
}

int process_rssi(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;

    wifi_ap_record_t ap_info;
    esp_wifi_sta_get_ap_info(&ap_info);

    ret = send_command_resp(if_type, CMD_STA_RSSI, CMD_RESPONSE_SUCCESS, (uint8_t *) &ap_info.rssi, sizeof(int8_t), sizeof(struct command_header));

    return ret;
}

int process_set_ip(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_set_ip_addr *cmd_set_ip;

    cmd_set_ip = (struct cmd_set_ip_addr *) payload;

    ip_address = le32toh(cmd_set_ip->ip);

    ret = send_command_resp(if_type, CMD_SET_IP_ADDR, CMD_RESPONSE_SUCCESS, NULL, 0, 0);

    return ret;
}

int process_wow_set(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    struct cmd_wow_config *cmd;

    cmd = (struct cmd_wow_config *)payload;

    wow.any = cmd->any;
    wow.disconnect = cmd->disconnect;
    wow.magic_pkt = cmd->magic_pkt;
    wow.four_way_handshake = cmd->four_way_handshake;
    wow.eap_identity_req = cmd->eap_identity_req;

    if (cmd->any) {
        wow.disconnect = 1;
        wow.magic_pkt = 1;
        wow.four_way_handshake = 1;
        wow.eap_identity_req = 1;
    }

    return send_command_resp(if_type, CMD_SET_WOW_CONFIG, CMD_RESPONSE_SUCCESS, NULL, 0, 0);
}

int process_set_time(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    struct cmd_set_time *cmd;
    struct timeval tv;

    cmd = (struct cmd_set_time *)payload;
    tv.tv_sec = cmd->sec;
    tv.tv_usec = cmd->usec;

    settimeofday(&tv, NULL); // Set time
    ESP_LOGI(TAG, "Updated firmware time sec=%lld, usec=%ld", tv.tv_sec, tv.tv_usec);

    return send_command_resp(if_type, CMD_SET_TIME, CMD_RESPONSE_SUCCESS, NULL, 0, 0);
}

int process_reg_set(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    struct cmd_reg_domain *cmd;

    cmd = (struct cmd_reg_domain *)payload;
    esp_wifi_set_country_code(cmd->country_code, false);

    return send_command_resp(if_type, CMD_SET_REG_DOMAIN, CMD_RESPONSE_SUCCESS, (uint8_t *)cmd->country_code, sizeof(cmd->country_code), 0);
}

int process_ota_start(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    uint16_t cmd_status = CMD_RESPONSE_SUCCESS;
    esp_err_t ret = ESP_OK;

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Failed to get next update partition");
        cmd_status = CMD_RESPONSE_FAIL;
        goto send_resp;
    }

    ret = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (ret) {
        ESP_LOGE(TAG, "OTA update failed in OTA begin");
        cmd_status = CMD_RESPONSE_FAIL;
        goto send_resp;
    }

    ESP_LOGI(TAG, "ESP OTA begin start");

send_resp:
    ret = send_command_resp(if_type, CMD_START_OTA_UPDATE, cmd_status, NULL, 0, 0);
    return ret;

}

int verify_ota_image_header(char *binary_image)
{
    esp_image_header_t *img_header = (esp_image_header_t *)binary_image;

    //verify image CHIP ID
    if (img_header->chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) {
        ESP_LOGE(TAG, "Firmware provided for ota has different chip id %x expected chip id %x", img_header->chip_id, CONFIG_IDF_FIRMWARE_CHIP_ID);
        return -1;
    }

    verify_ota = true;
    return 0;
}

int process_ota_write(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_ota_update_request *cmd;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;

    cmd = (struct cmd_ota_update_request *)(payload);

    if (!verify_ota) {
        ret = verify_ota_image_header(cmd->ota_binary);
        if (ret != 0) {
            goto fail;
        }
    }

    ret = esp_ota_write(handle, (const void *)cmd->ota_binary,
                        cmd->ota_binary_len);

fail:
    if (ret) {
        esp_ota_abort(handle);
        ESP_LOGE(TAG, "OTA update failed in OTA Write");
        cmd_status = CMD_RESPONSE_FAIL;
    }

    ret = send_command_resp(if_type, CMD_START_OTA_WRITE, cmd_status, NULL, 0, 0);
    return ret;
}

static void esp_reset_callback(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    esp_restart();
}

int process_ota_end(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
    TimerHandle_t xTimer = NULL;

    ret = esp_ota_end(handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "OTA update failed in end (%s)!", esp_err_to_name(ret));
        }
        cmd_status = CMD_RESPONSE_FAIL;
        goto fail;
    }

    /* set OTA partition for next boot */
    ret = esp_ota_set_boot_partition(update_partition);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(ret));
        cmd_status = CMD_RESPONSE_FAIL;
        goto fail;
    }

    xTimer = xTimerCreate("Reset Timer", RESET_TIMEOUT, pdFALSE, 0, esp_reset_callback);
    if (xTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer to restart system");
        cmd_status = CMD_RESPONSE_FAIL;
        goto fail;
    }

    ret = xTimerStart(xTimer, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to start timer to restart system");
        cmd_status = CMD_RESPONSE_FAIL;
        goto fail;
    }

    ESP_LOGE(TAG, "**** OTA updated successful, ESP will reboot in 5 sec ****");
fail:
    ESP_LOGI(TAG, "ESP OTA end");

    ret = send_command_resp(if_type, CMD_START_OTA_END, cmd_status, NULL, 0, 0);

    return ret;
}

int process_reg_get(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    char country_code[4];

    esp_wifi_get_country_code(country_code);
    return send_command_resp(if_type, CMD_SET_REG_DOMAIN, CMD_RESPONSE_SUCCESS, (uint8_t *)country_code, sizeof(country_code), 0);
}

int ieee80211_delete_node(uint8_t *mac);
int process_disconnect(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_disconnect *cmd_disconnect;
    wifi_mode_t wifi_mode = {0};

    cmd_disconnect = (struct cmd_disconnect *) payload;

    ESP_LOGI(TAG, "Disconnect request: reason [%d], interface=%d\n", cmd_disconnect->reason_code, if_type);

    ret = esp_wifi_get_mode(&wifi_mode);

    if (ret != ESP_OK) {
        goto resp;
    }
    if (if_type == ESP_STA_IF) {
        if (sta_init_flag && wifi_mode == WIFI_MODE_STA) {
            esp_wifi_deauthenticate_internal(cmd_disconnect->reason_code);
        }
    } else if (if_type == ESP_AP_IF) {
        if (softap_started &&
            (wifi_mode == WIFI_MODE_AP || wifi_mode == WIFI_MODE_APSTA)) {
            ieee80211_delete_node(cmd_disconnect->mac);
        }
    }

resp:
    ret = send_command_resp(if_type, CMD_DISCONNECT, CMD_RESPONSE_SUCCESS, NULL, 0, 0);

    return ret;
}

int process_auth_request(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_sta_auth *cmd_auth;
    wifi_config_t wifi_config = {0};
    uint32_t type = 0;
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE] = {0};
    uint8_t auth_type = 0;
    uint8_t msg_type = 0, *pos;
    uint8_t found_ssid = 0;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;

    cmd_auth = (struct cmd_sta_auth *) payload;

    esp_wifi_unset_appie_internal(WIFI_APPIE_RAM_STA_AUTH);
    /* Auth data generally present in WPA3 frame */
    if (cmd_auth->auth_data_len) {

        pos = (uint8_t *) cmd_auth->auth_data;
        msg_type = *pos;

        /* Set Auth IEs */
        esp_wifi_set_appie_internal(WIFI_APPIE_RAM_STA_AUTH, cmd_auth->auth_data + 4,
                                    cmd_auth->auth_data_len - 4, 0);
    }

    if (msg_type == 2) {
        /* WPA3 specific */
        ESP_LOGI(TAG, "AUTH Confirm\n");
        esp_wifi_send_auth_internal(0, cmd_auth->bssid, cmd_auth->auth_type, 2, 0);

    } else {
        wifi_scan_config_t params = {0};

        params.bssid = malloc(sizeof(cmd_auth->bssid));
        assert(params.bssid);

        memcpy(params.bssid, cmd_auth->bssid, sizeof(cmd_auth->bssid));
        params.scan_type = 0;
        params.show_hidden = true;

        if (cmd_auth->channel) {
            params.channel = cmd_auth->channel;
        }

        esp_wifi_scan_start(&params, true);

        free(params.bssid);
        ret = esp_wifi_scan_get_ap_records(&number, ap_info);
        if (ret) {
            ESP_LOGI(TAG, "Err: esp_wifi_scan_get_ap_records: %d\n", ret);
        }

        /* Register the Management frames */
        type = (1 << WLAN_FC_STYPE_ASSOC_RESP)
               | (1 << WLAN_FC_STYPE_REASSOC_RESP)
               | (1 << WLAN_FC_STYPE_AUTH)
               | (1 << WLAN_FC_STYPE_DEAUTH)
               | (1 << WLAN_FC_STYPE_DISASSOC);

        esp_wifi_register_mgmt_frame_internal(type, 0);

        /* ESP_LOG_BUFFER_HEXDUMP("BSSID", cmd_auth->bssid, MAC_ADDR_LEN, ESP_LOG_INFO); */
        for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < number); i++) {
            /*ESP_LOG_BUFFER_HEXDUMP("Next BSSID", ap_info[i].bssid, MAC_ADDR_LEN, ESP_LOG_INFO);
              ESP_LOGI(TAG, "ssid: %s, authmode: %u", ap_info[i].ssid, ap_info[i].authmode);*/
            if (memcmp(ap_info[i].bssid, cmd_auth->bssid, MAC_ADDR_LEN) == 0) {
                auth_type = ap_info[i].authmode;
                found_ssid = 1;
                break;
            }
        }

        if (!found_ssid) {
            ESP_LOGI(TAG, "AP not found to connect.");
            cmd_status = CMD_RESPONSE_FAIL;
            goto send_resp;
        }

        memcpy(wifi_config.sta.ssid, cmd_auth->ssid, MAX_SSID_LEN);
        /* ESP_LOGI(TAG, "ssid_found:%u Auth type scanned[%u], exp[%u] for ssid %s", found_ssid, auth_type, cmd_auth->auth_type, wifi_config.sta.ssid); */

        ESP_LOGI(TAG, "Connecting to %s, channel: %u [%d]", wifi_config.sta.ssid, cmd_auth->channel, auth_type);

        if (auth_type == WIFI_AUTH_WEP) {
            if (!cmd_auth->key_len) {
                ESP_LOGE(TAG, "WEP password not present");
            }
            memcpy(wifi_config.sta.password, cmd_auth->key, 27);
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WEP;
        } else if (auth_type == WIFI_AUTH_OWE) {
            wifi_config.sta.owe_enabled = 1;
        } else if (auth_type != WIFI_AUTH_OPEN) {
            memcpy(wifi_config.sta.password, DUMMY_PASSPHRASE, sizeof(DUMMY_PASSPHRASE));
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;;
        }

        ESP_LOGD(TAG, "AUTH type=%d password used=%s\n", auth_type, wifi_config.sta.password);
        memcpy(wifi_config.sta.bssid, cmd_auth->bssid, MAC_ADDR_LEN);
        wifi_config.sta.bssid_set = 1;

        wifi_config.sta.channel = cmd_auth->channel;

        /* Common handling for rest sec prot */
        ESP_LOGI(TAG, "AUTH Commit\n");
        ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        if (ret) {
            ESP_LOGE(TAG, "Failed to set wifi config: %d\n", ret);
            cmd_status = CMD_RESPONSE_FAIL;
            goto send_resp;
        }

        /* This API sends auth commit to AP */
        ret = esp_wifi_connect();
        if (ret) {
            ESP_LOGE(TAG, "Failed to connect wifi\n");
            cmd_status = CMD_RESPONSE_FAIL;
            goto send_resp;
        }
    }
    association_ongoing = 1;

send_resp:
    ret = send_command_resp(if_type, CMD_STA_AUTH, cmd_status, NULL, 0, 0);

    return ret;
}

int process_assoc_request(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_sta_assoc *cmd_assoc;
    uint16_t cmd_status = CMD_RESPONSE_SUCCESS;

    cmd_assoc = (struct cmd_sta_assoc *) payload;

    if (cmd_assoc->assoc_ie_len) {
        esp_wifi_unset_appie_internal(WIFI_APPIE_ASSOC_REQ);
        esp_wifi_set_appie_internal(WIFI_APPIE_ASSOC_REQ, cmd_assoc->assoc_ie,
                                    cmd_assoc->assoc_ie_len, 0);
    }
#define WLAN_FC_STYPE_ASSOC_REQ         0
#define WLAN_STATUS_SUCCESS 0
    esp_wifi_send_assoc_internal(WIFI_IF_STA, NULL, WLAN_FC_STYPE_ASSOC_REQ, WLAN_STATUS_SUCCESS);

    ret = send_command_resp(if_type, CMD_STA_ASSOC, cmd_status, NULL, 0, 0);

    if (!ret) {
        association_ongoing = 1;
    }

    return ret;
}

int process_sta_connect(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_sta_connect *cmd_connect;
    wifi_config_t wifi_config = {0};
    uint32_t type = 0;
    uint16_t cmd_status = CMD_RESPONSE_FAIL;

    type = (1 << WLAN_FC_STYPE_ASSOC_RESP)
           | (1 << WLAN_FC_STYPE_REASSOC_RESP)
           | (1 << WLAN_FC_STYPE_AUTH);
    esp_wifi_register_mgmt_frame_internal(type, 0);

    cmd_connect = (struct cmd_sta_connect *) payload;
    memcpy(wifi_config.sta.ssid, cmd_connect->ssid, MAX_SSID_LEN);
    if (!cmd_connect->is_auth_open) {
        ESP_LOGI(TAG, "Attempting secured connection");
        memcpy(wifi_config.sta.password, DUMMY_PASSPHRASE, sizeof(DUMMY_PASSPHRASE));
    } else {
        ESP_LOGI(TAG, "Attempting open connection");
    }
    memcpy(wifi_config.sta.bssid, cmd_connect->bssid, MAC_ADDR_LEN);
    wifi_config.sta.channel = cmd_connect->channel;
    ESP_LOGI(TAG, "%s, channel: %u", cmd_connect->ssid, cmd_connect->channel);

    if (cmd_connect->assoc_ie_len) {
        esp_wifi_unset_appie_internal(WIFI_APPIE_ASSOC_REQ);
        esp_wifi_set_appie_internal(WIFI_APPIE_ASSOC_REQ, cmd_connect->assoc_ie,
                                    cmd_connect->assoc_ie_len, 0);
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set wifi config: %d\n", ret);
        goto send_resp;
    }

    ret = esp_wifi_start();
    if (ret) {
        ESP_LOGE(TAG, "Failed to start wifi\n");
        goto send_resp;
    }

    ret = esp_wifi_connect();
    if (ret) {
        ESP_LOGE(TAG, "Failed to connect wifi\n");
        goto send_resp;
    }

    cmd_status = CMD_RESPONSE_SUCCESS;
send_resp:
    ret = send_command_resp(if_type, CMD_STA_CONNECT, cmd_status, NULL, 0, 0);

    if (!ret) {
        association_ongoing = 1;
    }

    return ret;
}

int process_deinit_interface(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    wifi_mode_t wifi_mode = {0};
    uint16_t cmd_status = CMD_RESPONSE_SUCCESS;

    if (if_type == ESP_STA_IF) {
        if (sta_init_flag && esp_wifi_get_mode(&wifi_mode) == 0) {
            esp_wifi_deauthenticate_internal(WIFI_REASON_AUTH_LEAVE);
        }
        esp_wifi_disconnect();
    }
    esp_wifi_scan_stop();
    esp_wifi_stop();

    ret = send_command_resp(if_type, CMD_DEINIT_INTERFACE, cmd_status, NULL, 0, 0);

    return ret;
}

int process_init_interface(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    uint16_t cmd_status = CMD_RESPONSE_FAIL;

    if (!sta_init_flag || (if_type == ESP_AP_IF && !softap_started)) {

        /* Use same MAC for AP and STA */
        esp_read_mac(dev_mac, ESP_MAC_WIFI_STA);

        if (if_type == ESP_AP_IF) {
            ESP_GOTO_ON_ERROR(esp_wifi_disconnect(), done, TAG, "Station Disconnect failed");
            ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), done, TAG, "Setting mode to APSTA failed");
            ESP_LOGI(TAG, "Setting APSTA mode");
            wifi_config_t wifi_config = {0};
            ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), done, TAG, "Set config for sta failed");
        } else if (if_type == ESP_STA_IF) {
            ESP_LOGI(TAG, "Setting STA mode");
            ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), done, TAG, "Setting mode to STA failed");
        } else {
            ESP_LOGE(TAG, "Invalid interface type");
            goto done;
        }

        ESP_GOTO_ON_ERROR(esp_wifi_start(), done, TAG, "Failed to start Wi-Fi");

    }
    cmd_status = CMD_RESPONSE_SUCCESS;
done:
    ret = send_command_resp(if_type, CMD_INIT_INTERFACE, cmd_status, NULL, 0, 0);
    if (ret != ESP_OK) {
        deinitialize_wifi();
    }

    return ret;
}

int process_get_mac(uint8_t if_type)
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;

    /*ESP_LOG_BUFFER_HEXDUMP(TAG, mac, MAC_ADDR_LEN, ESP_LOG_INFO);*/
    ret = send_command_resp(if_type, CMD_GET_MAC, cmd_status, dev_mac,
                            MAC_ADDR_LEN, sizeof(struct command_header));
    return ret;
}

int process_set_mac(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    wifi_interface_t wifi_if_type = 0;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
    struct cmd_config_mac_address *mac = (struct cmd_config_mac_address *) payload;

    if (if_type == ESP_STA_IF) {
        wifi_if_type = WIFI_IF_STA;
    } else {
        wifi_if_type = WIFI_IF_AP;
    }
    memcpy(dev_mac, mac->mac_addr, MAC_ADDR_LEN);

    ESP_LOGI(TAG, "Setting mac address \n");
    ret = esp_wifi_set_mac(wifi_if_type, mac->mac_addr);

    if (ret) {
        ESP_LOGE(TAG, "Failed to set mac address\n");
        cmd_status = CMD_RESPONSE_FAIL;
    }
    ret = send_command_resp(if_type, CMD_SET_MAC, cmd_status, (uint8_t *)mac->mac_addr,
                            MAC_ADDR_LEN, sizeof(struct command_header));
    return ret;
}

int process_set_default_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_key_operation *cmd = NULL;
    uint16_t cmd_status;

    /*ESP_LOGI(TAG, "%s:%u\n", __func__, __LINE__);*/

    cmd = (struct cmd_key_operation *) payload;
    if (!cmd) {
        ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
        cmd_status = CMD_RESPONSE_FAIL;
        goto SEND_CMD;
    }

    /* Firmware just responds to this request as success. */
    cmd_status = CMD_RESPONSE_SUCCESS;

SEND_CMD:
    ret = send_command_resp(if_type, CMD_SET_DEFAULT_KEY, cmd_status, NULL, 0, 0);

    return ret;
}

int process_del_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct wifi_sec_key * key = NULL;
    struct cmd_key_operation *cmd = NULL;
    uint16_t cmd_status = CMD_RESPONSE_SUCCESS;

    /*ESP_LOGI(TAG, "%s:%u\n", __func__, __LINE__);*/

    cmd = (struct cmd_key_operation *) payload;

    if (!cmd) {
        ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
        cmd_status = CMD_RESPONSE_FAIL;
        goto send_resp;
    }

    key = &cmd->key;

    if (!key->del) {
        ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
        cmd_status = CMD_RESPONSE_FAIL;
        goto send_resp;
    }

send_resp:
    ret = send_command_resp(if_type, CMD_DEL_KEY, cmd_status, NULL, 0, 0);

    return ret;
}

// hw_index = 8 for AP aid = 0
void esp_wifi_get_and_print_key(u8 hw_index)
{
    u8 idx;
    int algo;
    u8 addr[6];
    int keyindex;
    u8 key1[32];

    int ic_get_key(u8 * idx, int *algo, int *keyindex, u8 * addr, u8 hw_index, u8 * key, u8 key_len);
    ic_get_key(&idx, &algo, &keyindex, addr, hw_index, key1, 32);

    ESP_LOGI(TAG, "index=%d, algo=%d, keyindex=%d", idx, algo, keyindex);
    ESP_LOG_BUFFER_HEXDUMP("mac", addr, 6, ESP_LOG_INFO);
    ESP_LOG_BUFFER_HEXDUMP("key", key1, 32, ESP_LOG_INFO);
}

static int set_key_internal(void *data)
{
    wifi_interface_t iface = WIFI_IF_STA;
    struct cmd_key_operation *cmd = NULL;
    struct wifi_sec_key * key = NULL;
    uint8_t if_type;
    int ret;

    ESP_LOGI(TAG, "%s:%u\n", __func__, __LINE__);

    cmd = (struct cmd_key_operation *) data;

    if_type = cmd->header.reserved1;
    if (if_type == ESP_AP_IF) {
        iface = WIFI_IF_AP;
    }
    if (!cmd) {
        ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
        return -1;
    }

    key = &cmd->key;

    if (key->algo == WIFI_WPA_ALG_WEP40 || key->algo == WIFI_WPA_ALG_WEP104) {
        return 0;
    }
    if (key->index) {
        if (key->algo == WIFI_WPA_ALG_IGTK) {
            wifi_wpa_igtk_t igtk = {0};

            ESP_LOGI(TAG, "Setting iGTK [%ld]\n", key->index);

            memcpy(igtk.igtk, key->data, key->len);
            memcpy(igtk.pn, key->seq, key->seq_len);
            WPA_PUT_LE16(igtk.keyid, key->index);
            ret = esp_wifi_set_igtk_internal(iface, &igtk);
        } else {
            /* GTK */
            ESP_LOGI(TAG, "Setting GTK [%ld]\n", key->index);
            if (iface == WIFI_IF_AP) {
                ret = esp_wifi_set_ap_key_internal(key->algo, key->mac_addr, key->index,
                                                   key->data, key->len);
            } else {
                ret = esp_wifi_set_sta_key_internal(key->algo, key->mac_addr, key->index,
                                                    0, key->seq, key->seq_len, key->data, key->len,
                                                    KEY_FLAG_GROUP | KEY_FLAG_RX);
                esp_wifi_auth_done_internal();
            }
        }
    } else {
        /* PTK */
        ESP_LOGI(TAG, "Setting PTK algo=%ld index=%ld", key->algo, key->index);
        ESP_LOG_BUFFER_HEXDUMP("mac", key->mac_addr, 6, ESP_LOG_DEBUG);
        ESP_LOG_BUFFER_HEXDUMP("key", key->data, key->len, ESP_LOG_DEBUG);
        if (iface == WIFI_IF_AP) {
            ret = esp_wifi_set_ap_key_internal(key->algo, key->mac_addr, key->index,
                                               key->data, key->len);
        } else {
            ret = esp_wifi_set_sta_key_internal(key->algo, key->mac_addr, key->index,
                                                1, key->seq, key->seq_len, key->data, key->len,
                                                KEY_FLAG_PAIRWISE | KEY_FLAG_RX | KEY_FLAG_TX);
        }
    }

    if (ret) {
        ESP_LOGE(TAG, "%s:%u driver key set ret=%d\n", __func__, __LINE__, ret);
    }

    free(cmd);

    return ret;
}

static int wifi_set_keys(void *args)
{
    wifi_ipc_config_t cfg;

    cfg.fn = set_key_internal;
    cfg.arg = args;
    cfg.arg_size = 0;
    return esp_wifi_ipc_internal(&cfg, true);
}

int process_add_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    struct cmd_key_operation *cmd = NULL;
    uint8_t cmd_status;

    cmd = malloc(sizeof(*cmd));

    if (!cmd) {
        ESP_LOGE(TAG, "%s:%u memory allocation failed\n", __func__, __LINE__);
        cmd_status = CMD_RESPONSE_FAIL;
        goto SEND_CMD;
    }
    memcpy(cmd, payload, sizeof(*cmd));
    cmd->header.reserved1 = if_type;

    cmd_status = CMD_RESPONSE_SUCCESS;
    ret = wifi_set_keys(cmd);
    if (ret) {
        cmd_status = CMD_RESPONSE_FAIL;
    }

SEND_CMD:
    ret = send_command_resp(if_type, CMD_ADD_KEY, cmd_status, NULL, 0, 0);
    return ret;
}

int process_set_mode(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd_status = CMD_RESPONSE_FAIL;
    struct cmd_config_mode *mode = (struct cmd_config_mode *) payload;
    wifi_mode_t old_mode;

    ret = esp_wifi_get_mode(&old_mode);
    if (ret) {
        ESP_LOGE(TAG, "Failed to get mode");
    }

    if (old_mode == mode->mode) {
        ESP_LOGI(TAG, "old mode and new modes are same, return");
        goto send_ok;
    }
    ret = esp_wifi_stop();
    if (ret) {
        ESP_LOGE(TAG, "Failed to stop wifi\n");
    }
    
    if (mode->mode == WIFI_MODE_AP  || mode->mode == WIFI_MODE_APSTA) {
        ESP_LOGI(TAG, "Setting APSTA mode");
        ESP_GOTO_ON_ERROR(esp_wifi_set_mac(WIFI_IF_STA, dummy_mac), send_err, TAG, "Setting MAC on STA failed");
        ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), send_err, TAG, "Setting mode to APSTA failed");
        ESP_GOTO_ON_ERROR(esp_wifi_set_mac(WIFI_IF_AP, dev_mac), send_err, TAG, "Setting MAC on AP failed");
        wifi_config_t wifi_config = {0};
        ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), send_err, TAG, "Set config for sta failed");
    } else if (mode->mode == WIFI_MODE_STA) {
        ESP_LOGI(TAG, "Setting STA mode");
        ESP_GOTO_ON_ERROR(esp_wifi_set_mac(WIFI_IF_AP, dummy_mac2), send_err, TAG, "Setting MAC on AP failed");
        ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), send_err, TAG, "Setting mode to STA failed");
        ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, dev_mac));
    }

    ESP_GOTO_ON_ERROR(esp_wifi_start(), send_err, TAG, "Cannot start wifi");

send_ok:
    cmd_status = CMD_RESPONSE_SUCCESS;
send_err:
    return send_command_resp(if_type, CMD_SET_MODE, cmd_status, (uint8_t *)&mode->mode,
                            sizeof(uint16_t), sizeof(struct command_header));
}

int process_set_ie(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
    struct cmd_config_ie *ie = (struct cmd_config_ie *) payload;
    int type = 0;

    ESP_LOGI(TAG, "Setting IE type=%d len=%d\n", ie->ie_type, ie->ie_len);

    if (if_type != ESP_AP_IF) {
        cmd_status = CMD_RESPONSE_INVALID;
        goto send_resp;
    }
    if (ie->ie_type == IE_BEACON) {
        type = WIFI_APPIE_RAM_BEACON;
        ESP_LOG_BUFFER_HEXDUMP("BEACON", (uint8_t *) ie->ie, ie->ie_len, ESP_LOG_INFO);
    } else if (ie->ie_type == IE_BEACON_PROBE_HEAD) {
        type = WIFI_APPIE_RAM_BEACON_PROBE_HEAD;
        ESP_LOG_BUFFER_HEXDUMP("BEACON_PROBE_HEAD", (uint8_t *) ie->ie, ie->ie_len, ESP_LOG_INFO);
    } else if (ie->ie_type == IE_BEACON_PROBE_TAIL) {
        type = WIFI_APPIE_RAM_BEACON_PROBE_TAIL;
        ESP_LOG_BUFFER_HEXDUMP("BEACON_PROBE_TAIL", (uint8_t *) ie->ie, ie->ie_len, ESP_LOG_INFO);
    } else if (ie->ie_type == IE_PROBE_RESP) {
        type = WIFI_APPIE_RAM_PROBE_RSP;
        ESP_LOG_BUFFER_HEXDUMP("PROBE", (uint8_t *) ie->ie, ie->ie_len, ESP_LOG_INFO);
    } else if (ie->ie_type == IE_ASSOC_RESP) {
        type = WIFI_APPIE_ASSOC_RESP;
    } else {
        cmd_status = CMD_RESPONSE_INVALID;
        goto send_resp;
    }

    ret = esp_wifi_set_appie_internal(type, ie->ie, ie->ie_len, 0);

    if (ret) {
        cmd_status = CMD_RESPONSE_INVALID;
    }
send_resp:
    ret = send_command_resp(if_type, CMD_SET_IE, cmd_status, NULL, 0, 0);

    return ret;
}

static int send_mgmt_tx_done(uint8_t cmd_status, wifi_interface_t wifi_if_type, uint8_t *data, uint32_t len);

uint8_t *esp_wifi_get_eb_data(void *eb);
uint32_t esp_wifi_get_eb_data_len(void *eb);

void ieee80211_tx_mgt_cb(void *eb);

static void mgmt_txcb(void *eb)
{
    uint8_t cmd_status = CMD_RESPONSE_FAIL;

    if (esp_wifi_eb_tx_status_success_internal(eb)) {
        cmd_status = CMD_RESPONSE_SUCCESS;
    }

    uint8_t *data = esp_wifi_get_eb_data(eb);
    uint32_t len = esp_wifi_get_eb_data_len(eb);

    ieee80211_tx_mgt_cb(eb);
    if (!IS_BROADCAST_ADDR(data + 4)) {
        send_mgmt_tx_done(cmd_status, WIFI_IF_AP, data, len);
    }
    ESP_LOGD(TAG, "tx cb status=%d data_len=%ld\n", cmd_status, len);
    /* ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO); */
}

typedef enum {
    ETS_OK     = 0, /**< return successful in ets*/
    ETS_FAILED = 1, /**< return failed in ets*/
    ETS_PENDING = 2,
    ETS_BUSY = 3,
    ETS_CANCEL = 4,
} ETS_STATUS;
ETS_STATUS pp_unregister_tx_cb(uint8_t id);

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb);
int process_set_ap_config(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
    struct cmd_ap_config *ap_config = (struct cmd_ap_config *) payload;

    if (if_type != ESP_AP_IF) {
        cmd_status = CMD_RESPONSE_INVALID;
        goto send_resp;
    }

    wifi_config_t wifi_config = {0};

    memcpy(wifi_config.ap.ssid, ap_config->ap_config.ssid, ap_config->ap_config.ssid_len);
    /* set dummy config for data path */
    if (ap_config->ap_config.privacy) {
        memcpy(wifi_config.ap.password, "12345678", 8);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_WPA3_PSK;
    }
    wifi_config.ap.ssid_len = ap_config->ap_config.ssid_len;
    wifi_config.ap.channel = ap_config->ap_config.channel;
    wifi_config.ap.ssid_hidden = ap_config->ap_config.ssid_hidden;
    wifi_config.ap.beacon_interval = ap_config->ap_config.beacon_interval;

    ESP_LOGI(TAG, "ap config ssid=%s ssid_len=%d, pass=%s channel=%d authmode=%d hidden=%d bi=%d cipher=%d\n",
             wifi_config.ap.ssid, wifi_config.ap.ssid_len,
             wifi_config.ap.password,
             wifi_config.ap.channel, wifi_config.ap.authmode,
             wifi_config.ap.ssid_hidden, wifi_config.ap.beacon_interval,
             wifi_config.ap.pairwise_cipher);

    wifi_config.ap.max_connection = 8;

    wifi_mode_t old_mode;

    ret = esp_wifi_get_mode(&old_mode);
    if (ret) {
        ESP_LOGE(TAG, "Failed to get mode");
    }

    ret = esp_wifi_stop();

    if (old_mode != WIFI_MODE_APSTA) {
        ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    }
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

    if (ap_config->ap_config.inactivity_timeout) {
        ESP_LOGI(TAG, "inactivity_timeout=%d\n", ap_config->ap_config.inactivity_timeout);
        esp_wifi_set_inactive_time(WIFI_IF_AP, ap_config->ap_config.inactivity_timeout);
    }
#define WIFI_TXCB_MGMT_ID 2
    /* register for mgmt tx done */
    ret = pp_unregister_tx_cb(WIFI_TXCB_MGMT_ID);
    ESP_LOGI(TAG, "tx cb unregister ret=%d\n", ret);
    ret = esp_wifi_register_tx_cb_internal(mgmt_txcb, WIFI_TXCB_MGMT_ID);
    ESP_LOGI(TAG, "tx cb register ret=%d\n", ret);
    ret = esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t)wlan_ap_rx_callback);
#undef WIFI_TXCB_MGMT_ID
    ESP_ERROR_CHECK(esp_wifi_start());
send_resp:
    ret = send_command_resp(if_type, CMD_AP_CONFIG, cmd_status, NULL, 0, 0);

    return ret;
}

static int send_mgmt_tx_done(uint8_t cmd_status, wifi_interface_t wifi_if_type, uint8_t *data, uint32_t len)
{
    interface_buffer_handle_t buf_handle = {0};
    struct cmd_mgmt_tx *header;
    esp_err_t ret = ESP_OK;

    if (wifi_if_type == WIFI_IF_AP) {
        buf_handle.if_type = ESP_AP_IF;
    } else {
        buf_handle.if_type = ESP_STA_IF;
    }
    buf_handle.if_num = 0;
    buf_handle.payload_len = sizeof(struct cmd_mgmt_tx) + len;
    buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

    buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
    assert(buf_handle.payload);
    memset(buf_handle.payload, 0, buf_handle.payload_len);

    header = (struct cmd_mgmt_tx *) buf_handle.payload;

    header->header.cmd_code = CMD_MGMT_TX;
    header->header.len = 0;
    header->header.cmd_status = cmd_status;
    if (len > TX_DONE_PREFIX) {
        header->len = len - TX_DONE_PREFIX;
        memcpy(header->buf, data + TX_DONE_PREFIX, header->len);
    } else {
        header->len = len;
        memcpy(header->buf, data, header->len);
    }

    buf_handle.priv_buffer_handle = buf_handle.payload;
    buf_handle.free_buf_handle = free;

    ret = send_command_response(&buf_handle);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
        goto DONE;
    }

    return ESP_OK;
DONE:
    if (buf_handle.payload) {
        free(buf_handle.payload);
        buf_handle.payload = NULL;
    }

    return ret;
}

int ieee80211_send_mgmt_internal(wifi_interface_t wifi_if_type, uint8_t *buf, size_t len);

int process_mgmt_tx(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    esp_err_t ret = ESP_OK;
    wifi_interface_t wifi_if_type = WIFI_IF_AP;
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
    struct cmd_mgmt_tx *mgmt_tx = (struct cmd_mgmt_tx *) payload;

    if (if_type != ESP_AP_IF || !softap_started) {
        ESP_LOGE(TAG, "%s: err on wrong interface=%d\n", __func__, if_type);
        cmd_status = CMD_RESPONSE_INVALID;
        wifi_if_type = ESP_STA_IF;
        goto send_resp;
    }

    /* ESP_LOG_BUFFER_HEXDUMP("sending frame", mgmt_tx->buf, mgmt_tx->len, ESP_LOG_INFO); */

    ret = ieee80211_send_mgmt_internal(WIFI_IF_AP, mgmt_tx->buf, mgmt_tx->len);

    if (ret) {
        cmd_status = CMD_RESPONSE_INVALID;
        goto send_resp;
    }

    if (IS_BROADCAST_ADDR(mgmt_tx->buf + 4)) {
        ESP_LOGI(TAG, "%s: broadcast address, sending response immediately\n", __func__);
        goto send_resp;
    }
    /* send response in separate ctx once done */
    return 0;
send_resp:
    return send_mgmt_tx_done(cmd_status, wifi_if_type, NULL, 0);
}

int ieee80211_add_node(wifi_interface_t wifi_if_type, uint8_t *mac, uint16_t aid,
                       uint8_t *rates, uint8_t *htcap, uint8_t *vhtcap, uint8_t *hecap);

int add_station_node_ap(void *data)
{
    struct cmd_ap_add_sta_config *sta = (struct cmd_ap_add_sta_config *) data;

    ESP_LOG_BUFFER_HEXDUMP("mac", sta->sta_param.mac, 6, ESP_LOG_INFO);
    ESP_LOGI(TAG, "aid=%d\n", sta->sta_param.aid);

#define STA_FLAG_AUTHORIZED 1
    if (sta->sta_param.cmd != ADD_STA) {
        ESP_LOGD(TAG, "sta_flags_set=%ld, sta_flags_mask=%ld sta_modify_mask=%ld\n", sta->sta_param.sta_flags_set, sta->sta_param.sta_flags_mask, sta->sta_param.sta_modify_mask);
        if (sta->sta_param.sta_flags_set & BIT(STA_FLAG_AUTHORIZED)) {
            ESP_LOGI(TAG, "%s: authrizing station\n", __func__);
            esp_wifi_wpa_ptk_init_done_internal(sta->sta_param.mac);
        }
        ESP_LOGI(TAG, "%s: not station add cmd, handle later\n", __func__);
        return 0;
    } else {
        ESP_LOG_BUFFER_HEXDUMP("supported_rates", sta->sta_param.supported_rates, 12, ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEXDUMP("ht_rates", sta->sta_param.ht_caps, 28, ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEXDUMP("vht_rates", sta->sta_param.vht_caps, 14, ESP_LOG_INFO);
        ESP_LOG_BUFFER_HEXDUMP("he_rates", sta->sta_param.he_caps, 27, ESP_LOG_INFO);
    }
    return ieee80211_add_node(WIFI_IF_AP, sta->sta_param.mac, sta->sta_param.aid,
                              sta->sta_param.supported_rates[0] ? sta->sta_param.supported_rates : NULL,
                              sta->sta_param.ht_caps[0] ? sta->sta_param.ht_caps : NULL,
                              sta->sta_param.vht_caps[0] ? sta->sta_param.vht_caps : NULL,
                              sta->sta_param.he_caps[0] ? sta->sta_param.he_caps : NULL);
}

static int add_node_ap(void *args)
{
    wifi_ipc_config_t cfg;

    cfg.fn = add_station_node_ap;
    cfg.arg = args;
    cfg.arg_size = 0;
    return esp_wifi_ipc_internal(&cfg, true);
}

int process_ap_station(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
    uint8_t cmd_status = CMD_RESPONSE_SUCCESS;

    if (if_type != ESP_AP_IF) {
        ESP_LOGE(TAG, "%s: err on wrong interface=%d\n", __func__, if_type);
        cmd_status = CMD_RESPONSE_INVALID;
        goto send_resp;
    }

    ESP_LOGI(TAG, "%s:got station add command\n", __func__);
    /* ESP_LOG_BUFFER_HEXDUMP("sending frame", mgmt_tx->buf, mgmt_tx->len, ESP_LOG_INFO); */
    add_node_ap(payload);
    //esp_wifi_add_node(WIFI_IF_AP, );
send_resp:
    return send_command_resp(if_type, CMD_AP_STATION, cmd_status, NULL, 0, 0);
}
