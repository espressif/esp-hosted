/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2022 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 */

/** prevent recursive inclusion **/
#ifndef __RPC_API_H
#define __RPC_API_H

#include <stdbool.h>
#include "esp_hosted_rpc.pb-c.h"
#include "esp_wifi_types.h"

#ifdef __cplusplus
extern "C" {
#endif



#define SSID_LENGTH                          32
#define MAX_MAC_STR_LEN                      18
#define BSSID_LENGTH                         MAX_MAC_STR_LEN
#define BSSID_BYTES_SIZE                     6
#define PASSWORD_LENGTH                      64
#define STATUS_LENGTH                        14
#define VENDOR_OUI_BUF                       3

/*
#define SUCCESS 0
#define FAILURE -1
*/

#define CALLBACK_SET_SUCCESS                 0
#define CALLBACK_AVAILABLE                   0
#define CALLBACK_NOT_REGISTERED              -1
#define MSG_ID_OUT_OF_ORDER                  -2

#define MAX_FREE_BUFF_HANDLES                20

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

/* If request is already being served and
 * another request is pending, time period for
 * which new request will wait in seconds
 * */
#define WAIT_TIME_B2B_RPC_REQ               5
#define DEFAULT_RPC_RSP_TIMEOUT             30
#define DEFAULT_RPC_RSP_AP_SCAN_TIMEOUT     (60*3)


#define SUCCESS_STR                          "success"
#define FAILURE_STR                          "failure"
#define NOT_CONNECTED_STR                    "not_connected"

#define RPC_RX_QUEUE_SIZE 3
#define RPC_TX_QUEUE_SIZE 5

/*---- Control structures ----*/

typedef struct {
	int mode;
	uint8_t mac[BSSID_BYTES_SIZE];
} wifi_mac_t;

typedef struct {
	int mode;
} hosted_mode_t;

typedef struct {
	uint8_t ssid[SSID_LENGTH];
	uint8_t pwd[PASSWORD_LENGTH];
	uint8_t bssid[BSSID_LENGTH];
	bool is_wpa3_supported;
	int rssi;
	int channel;
	int encryption_mode;
	uint16_t listen_interval;
	char status[STATUS_LENGTH];
	char out_mac[MAX_MAC_STR_LEN];
} hosted_ap_config_t;

typedef struct {
	uint8_t ssid[SSID_LENGTH];
	uint8_t pwd[PASSWORD_LENGTH];
	int channel;
	int encryption_mode;
	int max_connections;
	bool ssid_hidden;
	wifi_bandwidth_t bandwidth;
	char out_mac[MAX_MAC_STR_LEN];
} hosted_softap_config_t;


typedef struct {
    uint8_t iface;
	wifi_config_t u;
} wifi_cfg_t;


/** @brief Parameters for an SSID scan. */
typedef struct {
	bool block;
	wifi_scan_config_t cfg;
	uint8_t cfg_set;
} wifi_scan_cfg_t;

typedef struct {
    //wifi_osi_funcs_t*      osi_funcs;              /**< WiFi OS functions */
    //wpa_crypto_funcs_t     wpa_crypto_funcs;       /**< WiFi station crypto functions when connect */
    int                    static_rx_buf_num;      /**< WiFi static RX buffer number */
    int                    dynamic_rx_buf_num;     /**< WiFi dynamic RX buffer number */
    int                    tx_buf_type;            /**< WiFi TX buffer type */
    int                    static_tx_buf_num;      /**< WiFi static TX buffer number */
    int                    dynamic_tx_buf_num;     /**< WiFi dynamic TX buffer number */
    int                    cache_tx_buf_num;       /**< WiFi TX cache buffer number */
    int                    csi_enable;             /**< WiFi channel state information enable flag */
    int                    ampdu_rx_enable;        /**< WiFi AMPDU RX feature enable flag */
    int                    ampdu_tx_enable;        /**< WiFi AMPDU TX feature enable flag */
    int                    amsdu_tx_enable;        /**< WiFi AMSDU TX feature enable flag */
    int                    nvs_enable;             /**< WiFi NVS flash enable flag */
    int                    nano_enable;            /**< Nano option for printf/scan family enable flag */
    int                    rx_ba_win;              /**< WiFi Block Ack RX window size */
    int                    wifi_task_core_id;      /**< WiFi Task Core ID */
    int                    beacon_max_len;         /**< WiFi softAP maximum length of the beacon */
    int                    mgmt_sbuf_num;          /**< WiFi management short buffer number, the minimum value is 6, the maximum value is 32 */
    uint64_t               feature_caps;           /**< Enables additional WiFi features and capabilities */
    bool                   sta_disconnected_pm;    /**< WiFi Power Management for station at disconnected status */
    int                    espnow_max_encrypt_num; /**< Maximum encrypt number of peers supported by espnow */
    int                    magic;                  /**< WiFi init magic number, it should be the last field */
} wifi_init_config_t;

typedef struct {
	//int count;
	int number;
	/* dynamic size */
	//wifi_scanlist_t *out_list;
	wifi_ap_record_t *out_list;
} wifi_scan_ap_list_t;

typedef struct {
	uint16_t aid;
} wifi_deauth_sta_t;

typedef struct {
	int ps_mode;
} wifi_power_save_t;

typedef struct {
	bool enable;
	wifi_vendor_ie_type_t type;
	wifi_vendor_ie_id_t idx;
	vendor_ie_data_t vnd_ie;
} wifi_softap_vendor_ie_t;

typedef struct {
	uint8_t *ota_data;
	uint32_t ota_data_len;
} ota_write_t;

typedef struct {
	int power;
} wifi_tx_power_t;

typedef struct {
	wifi_interface_t ifx;
	wifi_bandwidth_t bw;
} rpc_wifi_bandwidth_t;

typedef struct {
	uint8_t primary;
	wifi_second_chan_t second;
} rpc_wifi_channel_t;

typedef struct {
	char cc[3];
	bool ieee80211d_enabled;
} rpc_wifi_country_code;

typedef struct {
	wifi_interface_t ifx;
	uint8_t protocol_bitmap;
} rpc_wifi_protocol;

typedef struct {
	uint8_t mac[6];
	uint16_t aid;
} rpc_wifi_ap_get_sta_aid;

typedef struct {
	int rssi;
} rpc_wifi_sta_get_rssi;

typedef struct {
	/* event */
	uint32_t hb_num;
	/* Req */
	uint8_t enable;
	uint32_t duration;
} event_heartbeat_t;

typedef struct {
	int32_t reason;
	char mac[MAX_MAC_STR_LEN];
} event_station_disconn_t;

typedef struct {
	int32_t wifi_event_id;
} event_wifi_simple_t;

typedef struct Ctrl_cmd_t {
	/* msg type could be 1. req 2. resp 3. notification */
	uint8_t msg_type;

	/* control path protobuf msg number */
	uint16_t msg_id;

	/* statusof response or notification */
	uint8_t resp_event_status;

	void * rx_sem;

	union {
        wifi_init_config_t          wifi_init_config;
        wifi_cfg_t                  wifi_config;
		wifi_mac_t                  wifi_mac;
		hosted_mode_t               wifi_mode;

		hosted_ap_config_t          hosted_ap_config;

		hosted_softap_config_t      wifi_softap_config;
		wifi_softap_vendor_ie_t     wifi_softap_vendor_ie;
		//wifi_softap_conn_sta_list_t wifi_softap_con_sta;

		wifi_power_save_t           wifi_ps;

		ota_write_t                 ota_write;

		wifi_tx_power_t             wifi_tx_power;

		wifi_scan_cfg_t             wifi_scan_config;

		wifi_scan_ap_list_t         wifi_scan_ap_list;

		wifi_deauth_sta_t           wifi_deauth_sta;

		wifi_storage_t              wifi_storage;

		rpc_wifi_bandwidth_t        wifi_bandwidth;

		rpc_wifi_channel_t          wifi_channel;

		rpc_wifi_country_code       wifi_country_code;

		wifi_country_t              wifi_country;

		wifi_sta_list_t             wifi_ap_sta_list;

		rpc_wifi_ap_get_sta_aid     wifi_ap_get_sta_aid;

		rpc_wifi_sta_get_rssi       wifi_sta_get_rssi;

		rpc_wifi_protocol           wifi_protocol;

		event_heartbeat_t           e_heartbeat;

		event_station_disconn_t     e_sta_disconnected;

		event_wifi_simple_t         e_wifi_simple;

		wifi_event_ap_staconnected_t e_wifi_ap_staconnected;

		wifi_event_ap_stadisconnected_t e_wifi_ap_stadisconnected;

		wifi_event_sta_scan_done_t   e_wifi_sta_scan_done;

		wifi_event_sta_connected_t   e_wifi_sta_connected;

		wifi_event_sta_disconnected_t e_wifi_sta_disconnected;
	}u;

	/* By default this callback is set to NULL.
	 * When this callback is set by app while triggering request,
	 * it will be automatically called asynchronously
	 * by hosted control lib on receiving control response
	 * in this case app will not be waiting for response.
	 *
	 * Whereas, when this is not set i.e. is NULL, it is understood
	 * as synchronous response, and app after sending request,
	 * will wait till getting a response
	 */
	int (*rpc_rsp_cb)(struct Ctrl_cmd_t *data);

	/* Wait for timeout duration, if response not received,
	 * it will send timeout response.
	 * Default value for this time out is DEFAULT_RPC_RESP_TIMEOUT */
	int rsp_timeout_sec;

	/* rpc takes only one request at a time.
	 * If new request comes before previous command execution,
	 * wait for previous command execution for these many seconds, else return failure.
	 * Default: WAIT_TIME_B2B_RPC_REQ */
	int wait_prev_cmd_completion;

	/* assign the data pointer to free by lower layer.
	 * Ignored if assigned as NULL */
	void *app_free_buff_hdl;

	/* free handle to be registered
	 * Ignored if assigned as NULL */
	void (*app_free_buff_func)(void *app_free_buff_hdl);

	void *rpc_free_buff_hdls[MAX_FREE_BUFF_HANDLES];
	uint8_t n_rpc_free_buff_hdls;
} ctrl_cmd_t;


/* resp callback */
typedef int (*rpc_rsp_cb_t) (ctrl_cmd_t * resp);

/* event callback */
typedef int (*rpc_evt_cb_t) (ctrl_cmd_t * event);


/*---- Control API Function ----*/

/* This file contains hosted control library exposed APIs.
 * For detailed documentation, Please refer `../../../docs/common/ctrl_apis.md`
 *
 * As important note, application using these APIs, should clean
 *   1. allocated buffer within library are saved in `app_resp->app_free_buff_hdl`
 *   Please use `app_resp->app_free_buff_func` for freeing them.
 *   2. Response `ctrl_cmd_t *app_resp` is also allocated from library,
 *   need to free using g_h.funcs->_h_free() function.
 **/

/* Set control event callback
 *
 * when user sets event callback, user provided function pointer
 * will be registered with user function
 * If user does not register event callback,
 * events received from ESP32 will be dropped
 *
 * Inputs:
 * > event - Control Event ID
 * > event_cb - NULL - resets event callback
 *              Function pointer - Registers event callback
 * Returns:
 * > MSG_ID_OUT_OF_ORDER - If event is not registered with hosted control lib
 * > CALLBACK_SET_SUCCESS - Callback is set successful
 **/
int set_event_callback(int event, rpc_rsp_cb_t event_cb);

/* Reset control event callback
 *
 * when user sets event callback, user provided function pointer
 * will be registered with user function
 * If user does not register event callback,
 * events received from ESP32 will be dropped
 *
 * Inputs:
 * > event - Control Event ID
 *
 * Returns:
 * > MSG_ID_OUT_OF_ORDER - If event is not registered with hosted control lib
 * > CALLBACK_SET_SUCCESS - Callback is set successful
 **/
int reset_event_callback(int event);


/* Initialize hosted control library
 *
 * This is first step for application while using control path
 * This will allocate and instantiate hosted control library
 *
 * Returns:
 * > SUCCESS - 0
 * > FAILURE - -1
 **/
int init_rpc_lib(void);

/* De-initialize hosted control library
 *
 * This is last step for application while using control path
 * This will deallocate and cleanup hosted control library
 *
 * Returns:
 * > SUCCESS - 0
 * > FAILURE - -1
 **/
int deinit_rpc_lib(void);

/* Get the MAC address of station or softAP interface of ESP32 */
ctrl_cmd_t * wifi_get_mac(ctrl_cmd_t req);

/* Set MAC address of ESP32 interface for given wifi mode */
ctrl_cmd_t * wifi_set_mac(ctrl_cmd_t req);

/* Get Wi-Fi mode of ESP32 */
ctrl_cmd_t * wifi_get_mode(ctrl_cmd_t req);

/* Set the Wi-Fi mode of ESP32 */
ctrl_cmd_t * wifi_set_mode(ctrl_cmd_t req);

/* Set Wi-Fi power save mode of ESP32 */
ctrl_cmd_t * wifi_set_power_save_mode(ctrl_cmd_t req);

/* Get the Wi-Fi power save mode of ESP32 */
ctrl_cmd_t * wifi_get_power_save_mode(ctrl_cmd_t req);

///* Get list of available neighboring APs of ESP32 */
//ctrl_cmd_t * wifi_ap_scan_list(ctrl_cmd_t req);
//
///* Get the AP config to which ESP32 station is connected */
//ctrl_cmd_t * wifi_get_ap_config(ctrl_cmd_t req);
//
///* Set the AP config to which ESP32 station should connect to */
//ctrl_cmd_t * wifi_connect_ap(ctrl_cmd_t req);
//
///* Disconnect ESP32 station from AP */
//ctrl_cmd_t * wifi_disconnect_ap(ctrl_cmd_t req);
//
///* Set configuration of ESP32 softAP and start broadcasting */
//ctrl_cmd_t * wifi_start_softap(ctrl_cmd_t req);
//
///* Get configuration of ESP32 softAP */
//ctrl_cmd_t * wifi_get_softap_config(ctrl_cmd_t req);
//
///* Stop ESP32 softAP */
//ctrl_cmd_t * wifi_stop_softap(ctrl_cmd_t req);
//
///* Get list of connected stations to ESP32 softAP */
//ctrl_cmd_t * wifi_get_softap_connected_station_list(ctrl_cmd_t req);
//
///* Function set 802.11 Vendor-Specific Information Element.
// * It needs to get called before starting of ESP32 softAP */
//ctrl_cmd_t * wifi_set_vendor_specific_ie(ctrl_cmd_t req);

/* Sets maximum WiFi transmitting power at ESP32 */
ctrl_cmd_t * wifi_set_max_tx_power(ctrl_cmd_t req);

/* Gets current WiFi transmiting power at ESP32 */
ctrl_cmd_t * wifi_get_curr_tx_power(ctrl_cmd_t req);

/* Configure heartbeat event. Be default heartbeat is not enabled.
 * To enable heartbeats, user need to use this API in addition
 * to setting event callback for heartbeat event */
ctrl_cmd_t * config_heartbeat(ctrl_cmd_t req);

/* Performs an OTA begin operation for ESP32 which erases and
 * prepares existing flash partition for new flash writing */
ctrl_cmd_t * ota_begin(ctrl_cmd_t req);

/* Performs an OTA write operation for ESP32, It writes bytes from `ota_data`
 * buffer with `ota_data_len` number of bytes to OTA partition in flash. Number
 * of bytes can be small than size of complete binary to be flashed. In that
 * case, this caller is expected to repeatedly call this function till
 * total size written equals size of complete binary */
ctrl_cmd_t * ota_write(ctrl_cmd_t req);

/* Performs an OTA end operation for ESP32, It validates written OTA image,
 * sets newly written OTA partition as boot partition for next boot,
 * Creates timer which reset ESP32 after 5 sec */
ctrl_cmd_t * ota_end(ctrl_cmd_t req);

/* TODO: add descriptions */
ctrl_cmd_t * wifi_init(ctrl_cmd_t req);
ctrl_cmd_t * wifi_deinit(ctrl_cmd_t req);
ctrl_cmd_t * wifi_start(ctrl_cmd_t req);
ctrl_cmd_t * wifi_stop(ctrl_cmd_t req);
ctrl_cmd_t * wifi_connect(ctrl_cmd_t req);
ctrl_cmd_t * wifi_disconnect(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_config(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_config(ctrl_cmd_t req);
ctrl_cmd_t * wifi_scan_start(ctrl_cmd_t req);
ctrl_cmd_t * wifi_scan_stop(ctrl_cmd_t req);
ctrl_cmd_t * wifi_scan_get_ap_num(ctrl_cmd_t req);
ctrl_cmd_t * wifi_scan_get_ap_records(ctrl_cmd_t req);
ctrl_cmd_t * wifi_clear_ap_list(ctrl_cmd_t req);
ctrl_cmd_t * wifi_restore(ctrl_cmd_t req);
ctrl_cmd_t * wifi_clear_fast_connect(ctrl_cmd_t req);
ctrl_cmd_t * wifi_deauth_sta(ctrl_cmd_t req);
ctrl_cmd_t * wifi_sta_get_ap_info(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_ps(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_ps(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_storage(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_bandwidth(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_bandwidth(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_channel(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_channel(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_country_code(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_country_code(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_country(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_country(ctrl_cmd_t req);
ctrl_cmd_t * wifi_ap_get_sta_list(ctrl_cmd_t req);
ctrl_cmd_t * wifi_ap_get_sta_aid(ctrl_cmd_t req);
ctrl_cmd_t * wifi_sta_get_rssi(ctrl_cmd_t req);
ctrl_cmd_t * wifi_set_protocol(ctrl_cmd_t req);
ctrl_cmd_t * wifi_get_protocol(ctrl_cmd_t req);

/* Get the interface up for interface `iface` */
int interface_up(int sockfd, char* iface);

/* Get the interface down for interface `iface` */
int interface_down(int sockfd, char* iface);

/* Set ethernet interface MAC address `mac` to interface `iface` */
int set_hw_addr(int sockfd, char* iface, char* mac);

/* Create an endpoint for communication */
int create_socket(int domain, int type, int protocol, int *sock);

/* Close an endpoint of the communication */
int close_socket(int sock);

#ifdef __cplusplus
}
#endif

#endif
