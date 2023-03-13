/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2022 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 */

/** prevent recursive inclusion **/
#ifndef __CTRL_API_H
#define __CTRL_API_H

#include <stdbool.h>
#include "esp_hosted_config.pb-c.h"

#define SUCCESS                              0
#define FAILURE                              -1

#define SSID_LENGTH                          32
#define MAX_MAC_STR_LEN                      18
#define BSSID_LENGTH                         MAX_MAC_STR_LEN
#define PASSWORD_LENGTH                      64
#define STATUS_LENGTH                        14
#define VENDOR_OUI_BUF                       3

#define CALLBACK_SET_SUCCESS                 0
#define CALLBACK_AVAILABLE                   0
#define CALLBACK_NOT_REGISTERED              -1
#define MSG_ID_OUT_OF_ORDER                  -2

/* If request is already being served and
 * another request is pending, time period for
 * which new request will wait in seconds
 * */
#define WAIT_TIME_B2B_CTRL_REQ               5
#define DEFAULT_CTRL_RESP_TIMEOUT            30
#define DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT    (60*3)


#define SUCCESS_STR                          "success"
#define FAILURE_STR                          "failure"
#define NOT_CONNECTED_STR                    "not_connected"

/*---- Control structures ----*/

enum {
	CTRL_ERR_NOT_CONNECTED = 1,
	CTRL_ERR_NO_AP_FOUND,
	CTRL_ERR_INVALID_PASSWORD,
	CTRL_ERR_INVALID_ARGUMENT,
	CTRL_ERR_OUT_OF_RANGE,
	CTRL_ERR_MEMORY_FAILURE,
	CTRL_ERR_UNSUPPORTED_MSG,
	CTRL_ERR_INCORRECT_ARG,
	CTRL_ERR_PROTOBUF_ENCODE,
	CTRL_ERR_PROTOBUF_DECODE,
	CTRL_ERR_SET_ASYNC_CB,
	CTRL_ERR_TRANSPORT_SEND,
	CTRL_ERR_REQUEST_TIMEOUT,
	CTRL_ERR_REQ_IN_PROG,
	OUT_OF_RANGE
};

enum wifi_event_e {
    WIFI_EVENT_WIFI_READY = 0,           /**< ESP32 WiFi ready */
    WIFI_EVENT_SCAN_DONE,                /**< ESP32 finish scanning AP */
    WIFI_EVENT_STA_START,                /**< ESP32 station start */
    WIFI_EVENT_STA_STOP,                 /**< ESP32 station stop */
    WIFI_EVENT_STA_CONNECTED,            /**< ESP32 station connected to AP */
    WIFI_EVENT_STA_DISCONNECTED,         /**< ESP32 station disconnected from AP */
    WIFI_EVENT_STA_AUTHMODE_CHANGE,      /**< the auth mode of AP connected by ESP32 station changed */

    WIFI_EVENT_STA_WPS_ER_SUCCESS,       /**< ESP32 station wps succeeds in enrollee mode */
    WIFI_EVENT_STA_WPS_ER_FAILED,        /**< ESP32 station wps fails in enrollee mode */
    WIFI_EVENT_STA_WPS_ER_TIMEOUT,       /**< ESP32 station wps timeout in enrollee mode */
    WIFI_EVENT_STA_WPS_ER_PIN,           /**< ESP32 station wps pin code in enrollee mode */
    WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP,   /**< ESP32 station wps overlap in enrollee mode */

    WIFI_EVENT_AP_START,                 /**< ESP32 soft-AP start */
    WIFI_EVENT_AP_STOP,                  /**< ESP32 soft-AP stop */
    WIFI_EVENT_AP_STACONNECTED,          /**< a station connected to ESP32 soft-AP */
    WIFI_EVENT_AP_STADISCONNECTED,       /**< a station disconnected from ESP32 soft-AP */
    WIFI_EVENT_AP_PROBEREQRECVED,        /**< Receive probe request packet in soft-AP interface */

    WIFI_EVENT_FTM_REPORT,               /**< Receive report of FTM procedure */

    /* Add next events after this only */
    WIFI_EVENT_STA_BSS_RSSI_LOW,         /**< AP's RSSI crossed configured threshold */
    WIFI_EVENT_ACTION_TX_STATUS,         /**< Status indication of Action Tx operation */
    WIFI_EVENT_ROC_DONE,                 /**< Remain-on-Channel operation complete */

    WIFI_EVENT_STA_BEACON_TIMEOUT,       /**< ESP32 station beacon timeout */

    WIFI_EVENT_CONNECTIONLESS_MODULE_WAKE_INTERVAL_START,   /**< ESP32 connectionless module wake interval start */

    WIFI_EVENT_AP_WPS_RG_SUCCESS,       /**< Soft-AP wps succeeds in registrar mode */
    WIFI_EVENT_AP_WPS_RG_FAILED,        /**< Soft-AP wps fails in registrar mode */
    WIFI_EVENT_AP_WPS_RG_TIMEOUT,       /**< Soft-AP wps timeout in registrar mode */
    WIFI_EVENT_AP_WPS_RG_PIN,           /**< Soft-AP wps pin code in registrar mode */
    WIFI_EVENT_AP_WPS_RG_PBC_OVERLAP,   /**< Soft-AP wps overlap in registrar mode */

    WIFI_EVENT_MAX,                      /**< Invalid WiFi event ID */
};

enum AppMsgType_e {

	CTRL_MSGTYPE_INVALID = CTRL_MSG_TYPE__MsgType_Invalid,
	CTRL_REQ = CTRL_MSG_TYPE__Req,
	CTRL_RESP = CTRL_MSG_TYPE__Resp,
	CTRL_EVENT = CTRL_MSG_TYPE__Event,
	CTRL_MSGTYPE_MAX = CTRL_MSG_TYPE__MsgType_Max,

};

enum AppMsgId_e {

	CTRL_MSGID_INVALID = CTRL_MSG_ID__MsgId_Invalid,
	/*
	 ** Request Msgs *
	 */
	CTRL_REQ_BASE                      = CTRL_MSG_ID__Req_Base,
	CTRL_REQ_GET_MAC_ADDR              = CTRL_MSG_ID__Req_GetMACAddress, //0x65
	CTRL_REQ_SET_MAC_ADDR              = CTRL_MSG_ID__Req_SetMacAddress, //0x66
	CTRL_REQ_GET_WIFI_MODE             = CTRL_MSG_ID__Req_GetWifiMode,   //0x67
	CTRL_REQ_SET_WIFI_MODE             = CTRL_MSG_ID__Req_SetWifiMode,   //0x68

	CTRL_REQ_GET_AP_SCAN_LIST          = CTRL_MSG_ID__Req_GetAPScanList, //0x69
	CTRL_REQ_GET_AP_CONFIG             = CTRL_MSG_ID__Req_GetAPConfig,   //0x6a
	CTRL_REQ_CONNECT_AP                = CTRL_MSG_ID__Req_ConnectAP,     //0x6b
	CTRL_REQ_DISCONNECT_AP             = CTRL_MSG_ID__Req_DisconnectAP,  //0x6c

	CTRL_REQ_GET_SOFTAP_CONFIG         = CTRL_MSG_ID__Req_GetSoftAPConfig,            //0x6d
	CTRL_REQ_SET_SOFTAP_VND_IE         = CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE,  //0x6e
	CTRL_REQ_START_SOFTAP              = CTRL_MSG_ID__Req_StartSoftAP,                //0x6f
	CTRL_REQ_GET_SOFTAP_CONN_STA_LIST  = CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList,  //0x70
	CTRL_REQ_STOP_SOFTAP               = CTRL_MSG_ID__Req_StopSoftAP,                 //0x71

	CTRL_REQ_SET_PS_MODE               = CTRL_MSG_ID__Req_SetPowerSaveMode,   //0x72
	CTRL_REQ_GET_PS_MODE               = CTRL_MSG_ID__Req_GetPowerSaveMode,   //0x73

	CTRL_REQ_OTA_BEGIN                 = CTRL_MSG_ID__Req_OTABegin,           //0x74
	CTRL_REQ_OTA_WRITE                 = CTRL_MSG_ID__Req_OTAWrite,           //0x75
	CTRL_REQ_OTA_END                   = CTRL_MSG_ID__Req_OTAEnd,             //0x76

	CTRL_REQ_SET_WIFI_MAX_TX_POWER     = CTRL_MSG_ID__Req_SetWifiMaxTxPower,  //0x77
	CTRL_REQ_GET_WIFI_CURR_TX_POWER    = CTRL_MSG_ID__Req_GetWifiCurrTxPower, //0x78

	CTRL_REQ_CONFIG_HEARTBEAT          = CTRL_MSG_ID__Req_ConfigHeartbeat,    //0x79

	CTRL_REQ_WIFI_INIT                 = CTRL_MSG_ID__Req_WifiInit,           //0x7a
	CTRL_REQ_WIFI_DEINIT               = CTRL_MSG_ID__Req_WifiDeinit,         //0x7b
	CTRL_REQ_WIFI_START                = CTRL_MSG_ID__Req_WifiStart,          //0x7c
	CTRL_REQ_WIFI_STOP                 = CTRL_MSG_ID__Req_WifiStop,           //0x7d
	CTRL_REQ_WIFI_CONNECT              = CTRL_MSG_ID__Req_WifiConnect,        //0x7e
	CTRL_REQ_WIFI_DISCONNECT           = CTRL_MSG_ID__Req_WifiDisconnect,      //0x7f
	CTRL_REQ_WIFI_SET_CONFIG           = CTRL_MSG_ID__Req_WifiSetConfig,      //0x80
	CTRL_REQ_WIFI_GET_CONFIG           = CTRL_MSG_ID__Req_WifiGetConfig,      //0x81
	/*
	 * Add new control path command response before Req_Max
	 * and update Req_Max
	 */
	CTRL_REQ_MAX = CTRL_MSG_ID__Req_Max,
	/*
	 ** Response Msgs *
	 */
	CTRL_RESP_BASE                     = CTRL_MSG_ID__Resp_Base,
	CTRL_RESP_GET_MAC_ADDR             = CTRL_MSG_ID__Resp_GetMACAddress,  //0x65 -> 0xc9
	CTRL_RESP_SET_MAC_ADDRESS          = CTRL_MSG_ID__Resp_SetMacAddress,  //0x66 -> 0xca
	CTRL_RESP_GET_WIFI_MODE            = CTRL_MSG_ID__Resp_GetWifiMode,    //0x67 -> 0xcb
	CTRL_RESP_SET_WIFI_MODE            = CTRL_MSG_ID__Resp_SetWifiMode,    //0x68 -> 0xcc

	CTRL_RESP_GET_AP_SCAN_LIST         = CTRL_MSG_ID__Resp_GetAPScanList,  //0x69 -> 0xcd
	CTRL_RESP_GET_AP_CONFIG            = CTRL_MSG_ID__Resp_GetAPConfig,    //0x6a -> 0xce
	CTRL_RESP_CONNECT_AP               = CTRL_MSG_ID__Resp_ConnectAP,      //0x6b -> 0xdf
	CTRL_RESP_DISCONNECT_AP            = CTRL_MSG_ID__Resp_DisconnectAP,   //0x6c -> 0xd0

	CTRL_RESP_GET_SOFTAP_CONFIG        = CTRL_MSG_ID__Resp_GetSoftAPConfig,           //0x6d -> 0xd1
	CTRL_RESP_SET_SOFTAP_VND_IE        = CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE, //0x6e -> 0xd2
	CTRL_RESP_START_SOFTAP             = CTRL_MSG_ID__Resp_StartSoftAP,               //0x6f -> 0xd3
	CTRL_RESP_GET_SOFTAP_CONN_STA_LIST = CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList, //0x70 -> 0xd4
	CTRL_RESP_STOP_SOFTAP              = CTRL_MSG_ID__Resp_StopSoftAP,                //0x71 -> 0xd5

	CTRL_RESP_SET_PS_MODE              = CTRL_MSG_ID__Resp_SetPowerSaveMode, //0x72 -> 0xd6
	CTRL_RESP_GET_PS_MODE              = CTRL_MSG_ID__Resp_GetPowerSaveMode, //0x73 -> 0xd7

	CTRL_RESP_OTA_BEGIN                = CTRL_MSG_ID__Resp_OTABegin,         //0x74 -> 0xd8
	CTRL_RESP_OTA_WRITE                = CTRL_MSG_ID__Resp_OTAWrite,         //0x75 -> 0xd9
	CTRL_RESP_OTA_END                  = CTRL_MSG_ID__Resp_OTAEnd,           //0x76 -> 0xda

	CTRL_RESP_SET_WIFI_MAX_TX_POWER     = CTRL_MSG_ID__Resp_SetWifiMaxTxPower,  //0x77 -> 0xdb
	CTRL_RESP_GET_WIFI_CURR_TX_POWER    = CTRL_MSG_ID__Resp_GetWifiCurrTxPower, //0x78 -> 0xdc

	CTRL_RESP_CONFIG_HEARTBEAT          = CTRL_MSG_ID__Resp_ConfigHeartbeat,    //0x79 -> 0xdd

	CTRL_RESP_WIFI_INIT                 = CTRL_MSG_ID__Resp_WifiInit,           //0x7a -> 0xde
	CTRL_RESP_WIFI_DEINIT               = CTRL_MSG_ID__Resp_WifiDeinit,         //0x7b -> 0xdf
	CTRL_RESP_WIFI_START                = CTRL_MSG_ID__Resp_WifiStart,          //0x7c -> 0xe0
	CTRL_RESP_WIFI_STOP                 = CTRL_MSG_ID__Resp_WifiStop,           //0x7d -> 0xe1
	CTRL_RESP_WIFI_CONNECT              = CTRL_MSG_ID__Resp_WifiConnect,        //0x7e -> 0xe2
	CTRL_RESP_WIFI_DISCONNECT           = CTRL_MSG_ID__Resp_WifiDisconnect,     //0x7f -> 0xe3
	CTRL_RESP_WIFI_SET_CONFIG           = CTRL_MSG_ID__Resp_WifiSetConfig,      //0x80 -> 0xe4
	CTRL_RESP_WIFI_GET_CONFIG           = CTRL_MSG_ID__Resp_WifiGetConfig,      //0x81 -> 0xe5
	/*
	 * Add new control path comm       and response before Resp_Max
	 * and update Resp_Max
	 */
	CTRL_RESP_MAX = CTRL_MSG_ID__Resp_Max,
	/*
	 ** Events
	 */
	CTRL_EVENT_BASE            = CTRL_MSG_ID__Event_Base,
	CTRL_EVENT_ESP_INIT        = CTRL_MSG_ID__Event_ESPInit,
	CTRL_EVENT_HEARTBEAT       = CTRL_MSG_ID__Event_Heartbeat,
	CTRL_EVENT_STATION_DISCONNECT_FROM_AP = CTRL_MSG_ID__Event_StationDisconnectFromAP,
	CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP = CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP,
	CTRL_EVENT_WIFI_EVENT_NO_ARGS =  CTRL_MSG_ID__Event_WifiEventNoArgs,
	/*
	 * Add new control path command notification before Event_Max
	 * and update Event_Max
	 */
	CTRL_EVENT_MAX = CTRL_MSG_ID__Event_Max,
};

typedef enum {
	WIFI_MODE_NONE = 0,
	WIFI_MODE_STA,
	WIFI_MODE_AP,
	WIFI_MODE_APSTA,
	WIFI_MODE_MAX
} wifi_mode_t;

typedef enum {
	WIFI_CIPHER_TYPE_NONE = 0,   /**< the cipher type is none */
	WIFI_CIPHER_TYPE_WEP40,      /**< the cipher type is WEP40 */
	WIFI_CIPHER_TYPE_WEP104,     /**< the cipher type is WEP104 */
	WIFI_CIPHER_TYPE_TKIP,       /**< the cipher type is TKIP */
	WIFI_CIPHER_TYPE_CCMP,       /**< the cipher type is CCMP */
	WIFI_CIPHER_TYPE_TKIP_CCMP,  /**< the cipher type is TKIP and CCMP */
	WIFI_CIPHER_TYPE_AES_CMAC128,/**< the cipher type is AES-CMAC-128 */
	WIFI_CIPHER_TYPE_SMS4,       /**< the cipher type is SMS4 */
	WIFI_CIPHER_TYPE_GCMP,       /**< the cipher type is GCMP */
	WIFI_CIPHER_TYPE_GCMP256,    /**< the cipher type is GCMP-256 */
	WIFI_CIPHER_TYPE_AES_GMAC128,/**< the cipher type is AES-GMAC-128 */
	WIFI_CIPHER_TYPE_AES_GMAC256,/**< the cipher type is AES-GMAC-256 */
	WIFI_CIPHER_TYPE_UNKNOWN,    /**< the cipher type is unknown */
} wifi_cipher_type_t;

//TODO: unify wifi_ap_config_t and hosted_ap_config_t & softap configs
/* Strength of authmodes */
/* OPEN < WEP < WPA_PSK < OWE < WPA2_PSK = WPA_WPA2_PSK < WAPI_PSK < WPA2_ENTERPRISE < WPA3_PSK = WPA2_WPA3_PSK */
typedef enum {
    WIFI_AUTH_OPEN = 0,         /**< authenticate mode : open */
    WIFI_AUTH_WEP,              /**< authenticate mode : WEP */
    WIFI_AUTH_WPA_PSK,          /**< authenticate mode : WPA_PSK */
    WIFI_AUTH_WPA2_PSK,         /**< authenticate mode : WPA2_PSK */
    WIFI_AUTH_WPA_WPA2_PSK,     /**< authenticate mode : WPA_WPA2_PSK */
    WIFI_AUTH_WPA2_ENTERPRISE,  /**< authenticate mode : WPA2_ENTERPRISE */
    WIFI_AUTH_WPA3_PSK,         /**< authenticate mode : WPA3_PSK */
    WIFI_AUTH_WPA2_WPA3_PSK,    /**< authenticate mode : WPA2_WPA3_PSK */
    WIFI_AUTH_WAPI_PSK,         /**< authenticate mode : WAPI_PSK */
    WIFI_AUTH_OWE,              /**< authenticate mode : OWE */
    WIFI_AUTH_MAX
} wifi_auth_mode_t;

typedef enum {
	WIFI_BW_HT20 = 1,
	WIFI_BW_HT40,
} wifi_bandwidth_t;

typedef enum {
	WIFI_PS_NONE = 0,
	WIFI_PS_MIN_MODEM,
	WIFI_PS_MAX_MODEM,
	WIFI_PS_INVALID,
} wifi_ps_type_t;

typedef enum {
	WIFI_VND_IE_TYPE_BEACON = 0,
	WIFI_VND_IE_TYPE_PROBE_REQ,
	WIFI_VND_IE_TYPE_PROBE_RESP,
	WIFI_VND_IE_TYPE_ASSOC_REQ,
	WIFI_VND_IE_TYPE_ASSOC_RESP,
} wifi_vendor_ie_type_t;

typedef enum {
	WIFI_VND_IE_ID_0 = 0,
	WIFI_VND_IE_ID_1,
} wifi_vendor_ie_id_t;


typedef enum {
	WIFI_IF_STA = 0,
	WIFI_IF_AP,
	ESP_IF_ETH,
	WIFI_IF_MAX,
} wifi_interface_t;

typedef struct {
	/* Should be set to WIFI_VENDOR_IE_ELEMENT_ID (0xDD) */
	uint8_t element_id;
	/* Len of all bytes in the element data
	 * following this field. Minimum 4 */
	uint8_t length;
	/* Vendor identifier (OUI) */
	uint8_t vendor_oui[VENDOR_OUI_BUF];
	/* Vendor-specific OUI type */
	uint8_t vendor_oui_type;
	/*length of payload field*/
	uint16_t payload_len;
	/* Payload. Length is equal to value in 'length' field, minus 4 */
	uint8_t* payload;

} vendor_ie_data_t;

typedef struct {
	uint8_t ssid[SSID_LENGTH];
	uint8_t bssid[BSSID_LENGTH];
	int rssi;
	int channel;
	int encryption_mode;
} wifi_scanlist_t;

typedef struct {
	uint8_t bssid[BSSID_LENGTH];
	int rssi;
} wifi_connected_stations_list_t;

typedef struct {
	int mode;
	char mac[MAX_MAC_STR_LEN];
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


/** Configuration structure for Protected Management Frame */
typedef struct {
	bool capable;            /**< Deprecated variable. Device will always connect in PMF mode if other device also advertizes PMF capability. */
	bool required;           /**< Advertizes that Protected Management Frame is required. Device will not associate to non-PMF capable devices. */
} wifi_pmf_config_t;

/** Configuration for SAE PWE derivation */
typedef enum {
	WPA3_SAE_PWE_UNSPECIFIED = 0,
	WPA3_SAE_PWE_HUNT_AND_PECK,
	WPA3_SAE_PWE_HASH_TO_ELEMENT,
	WPA3_SAE_PWE_BOTH,
} wifi_sae_pwe_method_t;

typedef enum {
	WIFI_FAST_SCAN = 0,                   /**< Do fast scan, scan will end after find SSID match AP */
	WIFI_ALL_CHANNEL_SCAN,                /**< All channel scan, scan will end after scan all the channel */
} wifi_scan_method_t;

/** @brief Structure describing parameters for a WiFi fast scan */
typedef struct {
	int8_t              rssi;             /**< The minimum rssi to accept in the fast scan mode */
	wifi_auth_mode_t    authmode;         /**< The weakest authmode to accept in the fast scan mode
                                               Note: Incase this value is not set and password is set as per WPA2 standards(password len >= 8), it will be defaulted to WPA2 and device won't connect to deprecated WEP/WPA networks. Please set authmode threshold as WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK to connect to WEP/WPA networks */
} wifi_scan_threshold_t;

typedef enum {
	WIFI_CONNECT_AP_BY_SIGNAL = 0,        /**< Sort match AP in scan list by RSSI */
	WIFI_CONNECT_AP_BY_SECURITY,          /**< Sort match AP in scan list by security mode */
} wifi_sort_method_t;

/** @brief Soft-AP configuration settings for the ESP32 */
typedef struct {
	uint8_t ssid[32];           /**< SSID of ESP32 soft-AP. If ssid_len field is 0, this must be a Null terminated string. Otherwise, length is set according to ssid_len. */
	uint8_t password[64];       /**< Password of ESP32 soft-AP. */
	uint8_t ssid_len;           /**< Optional length of SSID field. */
	uint8_t channel;            /**< Channel of ESP32 soft-AP */
	wifi_auth_mode_t authmode;  /**< Auth mode of ESP32 soft-AP. Do not support AUTH_WEP in soft-AP mode */
	uint8_t ssid_hidden;        /**< Broadcast SSID or not, default 0, broadcast the SSID */
	uint8_t max_connection;     /**< Max number of stations allowed to connect in */
	uint16_t beacon_interval;   /**< Beacon interval which should be multiples of 100. Unit: TU(time unit, 1 TU = 1024 us). Range: 100 ~ 60000. Default value: 100 */
	wifi_cipher_type_t pairwise_cipher;   /**< pairwise cipher of SoftAP, group cipher will be derived using this. cipher values are valid starting from WIFI_CIPHER_TYPE_TKIP, enum values before that will be considered as invalid and default cipher suites(TKIP+CCMP) will be used. Valid cipher suites in softAP mode are WIFI_CIPHER_TYPE_TKIP, WIFI_CIPHER_TYPE_CCMP and WIFI_CIPHER_TYPE_TKIP_CCMP. */
	bool ftm_responder;         /**< Enable FTM Responder mode */
	wifi_pmf_config_t pmf_cfg;  /**< Configuration for Protected Management Frame */
} wifi_ap_config_t;

/** @brief STA configuration settings for the ESP32 */
typedef struct {
	uint8_t ssid[32];      /**< SSID of target AP. */
	uint8_t password[64];  /**< Password of target AP. */
	wifi_scan_method_t scan_method;    /**< do all channel scan or fast scan */
	bool bssid_set;        /**< whether set MAC address of target AP or not. Generally, station_config.bssid_set needs to be 0; and it needs to be 1 only when users need to check the MAC address of the AP.*/
	uint8_t bssid[6];     /**< MAC address of target AP*/
	uint8_t channel;       /**< channel of target AP. Set to 1~13 to scan starting from the specified channel before connecting to AP. If the channel of AP is unknown, set it to 0.*/
	uint16_t listen_interval;   /**< Listen interval for ESP32 station to receive beacon when WIFI_PS_MAX_MODEM is set. Units: AP beacon intervals. Defaults to 3 if set to 0. */
	wifi_sort_method_t sort_method;    /**< sort the connect AP in the list by rssi or security mode */
	wifi_scan_threshold_t  threshold;     /**< When sort_method is set, only APs which have an auth mode that is more secure than the selected auth mode and a signal stronger than the minimum RSSI will be used. */
	wifi_pmf_config_t pmf_cfg;    /**< Configuration for Protected Management Frame. Will be advertized in RSN Capabilities in RSN IE. */
	uint32_t rm_enabled:1;        /**< Whether Radio Measurements are enabled for the connection */
	uint32_t btm_enabled:1;       /**< Whether BSS Transition Management is enabled for the connection */
	uint32_t mbo_enabled:1;       /**< Whether MBO is enabled for the connection */
	uint32_t ft_enabled:1;        /**< Whether FT is enabled for the connection */
	uint32_t owe_enabled:1;       /**< Whether OWE is enabled for the connection */
	uint32_t transition_disable:1;      /**< Whether to enable transition disable feature */
	uint32_t reserved:26;         /**< Reserved for future feature set */
	wifi_sae_pwe_method_t sae_pwe_h2e;     /**< Whether SAE hash to element is enabled */
	uint8_t failure_retry_cnt;    /**< Number of connection retries station will do before moving to next AP. scan_method should be set as WIFI_ALL_CHANNEL_SCAN to use this config. Note: Enabling this may cause connection time to increase incase best AP doesn't behave properly. */
} wifi_sta_config_t;

/** @brief Configuration data for ESP32 AP or STA.
 *
 * The usage of this union (for ap or sta configuration) is determined by the accompanying
 * interface argument passed to esp_wifi_set_config() or esp_wifi_get_config()
 *
 */
typedef struct {
    uint8_t iface;
    union {
        wifi_ap_config_t  ap;  /**< configuration of AP */
        wifi_sta_config_t sta; /**< configuration of STA */
    };
} wifi_config_t;

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
	int count;
	/* dynamic size */
	wifi_scanlist_t *out_list;
} wifi_ap_scan_list_t;

typedef struct {
	int count;
	/* dynamic list*/
	wifi_connected_stations_list_t *out_list;
} wifi_softap_conn_sta_list_t;

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

	union {
        wifi_init_config_t          wifi_init_config;
        wifi_config_t               wifi_config;
		wifi_mac_t                  wifi_mac;
		hosted_mode_t               wifi_mode;

		wifi_ap_scan_list_t         wifi_ap_scan;
		hosted_ap_config_t          hosted_ap_config;

		hosted_softap_config_t      wifi_softap_config;
		wifi_softap_vendor_ie_t     wifi_softap_vendor_ie;
		wifi_softap_conn_sta_list_t wifi_softap_con_sta;

		wifi_power_save_t           wifi_ps;

		ota_write_t                 ota_write;

		wifi_tx_power_t             wifi_tx_power;

		event_heartbeat_t           e_heartbeat;

		event_station_disconn_t     e_sta_disconnected;

		event_wifi_simple_t         e_wifi_simple;
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
	int (*ctrl_resp_cb)(struct Ctrl_cmd_t *data);

	/* Wait for timeout duration, if response not received,
	 * it will send timeout response.
	 * Default value for this time out is DEFAULT_CTRL_RESP_TIMEOUT */
	int cmd_timeout_sec;

	/* assign the data pointer to free by lower layer.
	 * Ignored if assigned as NULL */
	void *free_buffer_handle;

	/* free handle to be registered
	 * Ignored if assigned as NULL */
	void (*free_buffer_func)(void *free_buffer_handle);
} ctrl_cmd_t;


/* resp callback */
typedef int (*ctrl_resp_cb_t) (ctrl_cmd_t * resp);

/* event callback */
typedef int (*ctrl_event_cb_t) (ctrl_cmd_t * event);


/*---- Control API Function ----*/

/* This file contains hosted control library exposed APIs.
 * For detailed documentation, Please refer `../../../docs/common/ctrl_apis.md`
 *
 * As important note, application using these APIs, should clean
 *   1. allocated buffer within library are saved in `app_resp->free_buffer_handle`
 *   Please use `app_resp->free_buffer_func` for freeing them.
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
 * > event - Control Event ID from `AppMsgId_e`
 * > event_cb - NULL - resets event callback
 *              Function pointer - Registers event callback
 * Returns:
 * > MSG_ID_OUT_OF_ORDER - If event is not registered with hosted control lib
 * > CALLBACK_SET_SUCCESS - Callback is set successful
 **/
int set_event_callback(int event, ctrl_resp_cb_t event_cb);

/* Reset control event callback
 *
 * when user sets event callback, user provided function pointer
 * will be registered with user function
 * If user does not register event callback,
 * events received from ESP32 will be dropped
 *
 * Inputs:
 * > event - Control Event ID from `AppMsgId_e`
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
int init_hosted_control_lib(void);

/* De-initialize hosted control library
 *
 * This is last step for application while using control path
 * This will deallocate and cleanup hosted control library
 *
 * Returns:
 * > SUCCESS - 0
 * > FAILURE - -1
 **/
int deinit_hosted_control_lib(void);

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

/* Get list of available neighboring APs of ESP32 */
ctrl_cmd_t * wifi_ap_scan_list(ctrl_cmd_t req);

/* Get the AP config to which ESP32 station is connected */
ctrl_cmd_t * wifi_get_ap_config(ctrl_cmd_t req);

/* Set the AP config to which ESP32 station should connect to */
ctrl_cmd_t * wifi_connect_ap(ctrl_cmd_t req);

/* Disconnect ESP32 station from AP */
ctrl_cmd_t * wifi_disconnect_ap(ctrl_cmd_t req);

/* Set configuration of ESP32 softAP and start broadcasting */
ctrl_cmd_t * wifi_start_softap(ctrl_cmd_t req);

/* Get configuration of ESP32 softAP */
ctrl_cmd_t * wifi_get_softap_config(ctrl_cmd_t req);

/* Stop ESP32 softAP */
ctrl_cmd_t * wifi_stop_softap(ctrl_cmd_t req);

/* Get list of connected stations to ESP32 softAP */
ctrl_cmd_t * wifi_get_softap_connected_station_list(ctrl_cmd_t req);

/* Function set 802.11 Vendor-Specific Information Element.
 * It needs to get called before starting of ESP32 softAP */
ctrl_cmd_t * wifi_set_vendor_specific_ie(ctrl_cmd_t req);

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

#endif
