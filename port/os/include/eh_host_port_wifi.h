/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PORT_ESP_HOSTED_HOST_WIFI_CONFIG_H__
#define __PORT_ESP_HOSTED_HOST_WIFI_CONFIG_H__

/* On non-IDF builds, supply ESP_IDF_VERSION_VAL ourselves so version-
 * gated IDF code sees all features "present". */
#if defined(__has_include)
#  if __has_include("esp_idf_version.h")
#    include "esp_idf_version.h"
#  endif
#endif

#ifndef ESP_IDF_VERSION_VAL
#  define ESP_IDF_VERSION_VAL(major, minor, patch) \
       (((major) << 16) | ((minor) << 8) | (patch))
#endif

#ifndef ESP_IDF_VERSION
   /* Linux / non-IDF default: pretend we're on the newest IDF so every
    * feature gate compiled below evaluates the "latest" branch. CONFIG_*
    * symbols (CONFIG_ESP_WIFI_* plus Hosted Wi-Fi feature toggles) that gate
    * actual code paths still drive whether the code is compiled in. */
#  define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(99, 0, 0)
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT && CONFIG_SLAVE_SOC_WIFI_HE_SUPPORT
  #define EH_HOST_WIFI_HE_SUPPORT 1
#else
  #define EH_HOST_WIFI_HE_SUPPORT 0
#endif

// HE support (structs, API) changed after ESP-IDF v5.3
#if EH_HOST_WIFI_HE_SUPPORT && (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 3, 0))
  #define EH_HOST_WIFI_HE_GREATER_THAN_ESP_IDF_5_3 1
#else
  #define EH_HOST_WIFI_HE_GREATER_THAN_ESP_IDF_5_3 0
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  /* dual band API support available */
  #define EH_HOST_WIFI_DUALBAND_SUPPORT 1
#else
  #define EH_HOST_WIFI_DUALBAND_SUPPORT 0
#endif

#ifdef CONFIG_ESP_WIFI_REMOTE_EAP_ENABLED
  #define EH_HOST_WIFI_ENTERPRISE_SUPPORT 1
#else
  #define EH_HOST_WIFI_ENTERPRISE_SUPPORT 0
#endif

/* ESP-IDF 5.5.0 breaking change: reserved/he_reserved renamed to reserved1/reserved2 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  #define EH_HOST_WIFI_NEW_RESERVED_FIELD_NAMES 1
  #define EH_HOST_PRESENT_IN_ESP_IDF_5_5_0      1
#else
  #define EH_HOST_WIFI_NEW_RESERVED_FIELD_NAMES 0
  #define EH_HOST_PRESENT_IN_ESP_IDF_5_5_0      0
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  #define EH_HOST_PRESENT_IN_ESP_IDF_5_4_0      1
#else
  #define EH_HOST_PRESENT_IN_ESP_IDF_5_4_0      0
#endif

/* User-controllable reserved field decoding - works regardless of IDF version */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_DECODE_RESERVED_FIELD
  #define EH_HOST_DECODE_WIFI_RESERVED_FIELD 1
#else
  #define EH_HOST_DECODE_WIFI_RESERVED_FIELD 0
#endif

/*
 * wifi_twt_config_t::twt_enable_keep_alive only found in
 * IDF v5.3.2 and above
 */
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 3, 1)
#define EH_HOST_GOT_TWT_ENABLE_KEEP_ALIVE 1
#else
#define EH_HOST_GOT_TWT_ENABLE_KEEP_ALIVE 0
#endif

/* wifi_ap_config_t::transition_disable only found in
 * IDF v5.3.3 and above, or
 * IDF v5.4.1 and above
 */
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 3) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 0)) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 1))
  #define EH_HOST_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE 1
#else
  #define EH_HOST_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE 0
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  #define EH_HOST_PRESENT_IN_ESP_IDF_6_0_0      1
#else
  #define EH_HOST_PRESENT_IN_ESP_IDF_6_0_0      0
#endif

#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 4) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 0)) || \
     (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 3) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)) || \
     (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 1)))
  #define EH_HOST_GOT_SET_EAP_METHODS_API 1
#else
  #define EH_HOST_GOT_SET_EAP_METHODS_API 0
#endif

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 4) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 0)) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 3))
  #define EH_HOST_GOT_EAP_SET_DOMAIN_NAME 1
#else
  #define EH_HOST_GOT_EAP_SET_DOMAIN_NAME 0
#endif

#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 3))
#define EH_HOST_GOT_EAP_OKC_SUPPORT 0
#else
#define EH_HOST_GOT_EAP_OKC_SUPPORT 1
#endif
/**
 * Wi-Fi Easy Connect (DPP) events is returned to user via
 * Supplicant Callback or Wi-Fi DPP events,
 * depending on IDF version
 *
 * IDF v6.0 and above only support Wi-Fi DPP events
 * IDF v5.5 support Wi-Fi and Supplicant DPP events
 * earlier versions support only Supplicant DPP events
 */
// Supplicant Callback DPP Events: still available, but deprecated
#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0))
#define EH_HOST_SUPP_DPP_SUPPORT 1
#else
#define EH_HOST_SUPP_DPP_SUPPORT 0
#endif

// Wi-Fi DPP Events: only in IDF v5.5 and above
#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0))
#define EH_HOST_WIFI_DPP_SUPPORT 1
#else
#define EH_HOST_WIFI_DPP_SUPPORT 0
#endif

// for generic DPP support
#if EH_HOST_SUPP_DPP_SUPPORT || EH_HOST_WIFI_DPP_SUPPORT
#define EH_HOST_DPP_SUPPORT 1
#else
#define EH_HOST_DPP_SUPPORT 0
#endif

// this sets the maximum length of the URI we can receive to generate
// the QR code
#if EH_HOST_DPP_SUPPORT
#define EH_HOST_DPP_URI_LEN_MAX CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP_URI_LEN_MAX
#endif

#endif /* __ESP_HOSTED_WIFI_CONFIG_H__ */
