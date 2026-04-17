/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * eh_cp_idf_compat.h
 *
 * Centralized IDF version compatibility macros.
 * Included from eh_cp_master_config.h — all extensions get these.
 *
 * Convention: EH_CP_* prefix, evaluates to 1 (available) or 0 (not).
 * Migrated from legacy H_* macros (see macro_migration_map.md).
 */

#ifndef ESP_HOSTED_CP_IDF_COMPAT_H
#define ESP_HOSTED_CP_IDF_COMPAT_H

#include "esp_idf_version.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * IDF version gates
 * ═══════════════════════════════════════════════════════════════════════════ */

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
#  define EH_CP_IDF_GE_5_4                      1
#else
#  define EH_CP_IDF_GE_5_4                      0
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
#  define EH_CP_IDF_GE_5_5                      1
#else
#  define EH_CP_IDF_GE_5_5                      0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * OTA
 * ═══════════════════════════════════════════════════════════════════════════ */

/* esp_ota_check_image_validity: only available in IDF v6.0+ */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#  define EH_CP_OTA_CHECK_IMAGE_VALIDITY         1
#else
#  define EH_CP_OTA_CHECK_IMAGE_VALIDITY         0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * WiFi HE (802.11ax)
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_SOC_WIFI_HE_SUPPORT
#  define EH_CP_SOC_WIFI_HE                      1
#else
#  define EH_CP_SOC_WIFI_HE                      0
#endif

/* HE API changed after IDF v5.3 */
#if EH_CP_SOC_WIFI_HE && (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 3, 0))
#  define EH_CP_WIFI_HE_GT_IDF_5_3              1
#else
#  define EH_CP_WIFI_HE_GT_IDF_5_3              0
#endif

/* wifi_twt_config_t::twt_enable_keep_alive: IDF v5.3.2+ */
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 3, 1)
#  define EH_CP_WIFI_GOT_TWT_KEEP_ALIVE         1
#else
#  define EH_CP_WIFI_GOT_TWT_KEEP_ALIVE         0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * WiFi API field availability
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ESP-IDF 5.5.0: renamed reserved fields to reserved1/reserved2 */
#if EH_CP_IDF_GE_5_5
#  define EH_CP_WIFI_NEW_RESERVED_FIELDS         1
#else
#  define EH_CP_WIFI_NEW_RESERVED_FIELDS         0
#endif

/* wifi_ap_config_t::transition_disable: IDF v5.3.3+ or v5.4.1+ */
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 3)) || \
    (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 1))
#  define EH_CP_WIFI_GOT_AP_TRANSITION_DISABLE   0
#else
#  define EH_CP_WIFI_GOT_AP_TRANSITION_DISABLE   1
#endif

/* WIFI_ENABLE_CACHE_TX_BUFFER: IDF < 5.3.3 fallback */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 3)
#  ifndef WIFI_ENABLE_CACHE_TX_BUFFER
#    define WIFI_ENABLE_CACHE_TX_BUFFER          WIFI_ENABLE_SPIRAM
#  endif
#endif

/* SPI slave enable/disable API: IDF >= 5.5.0 */
#if EH_CP_IDF_GE_5_5
#  define EH_CP_IDF_SPI_SLAVE_EN_DIS             1
#else
#  define EH_CP_IDF_SPI_SLAVE_EN_DIS             0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * WiFi Enterprise
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT
#  define EH_CP_WIFI_ENTERPRISE                  1
#else
#  define EH_CP_WIFI_ENTERPRISE                  0
#endif

/* esp_eap_client_set_eap_methods: compound IDF version check */
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 4)) || \
    (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 3)) || \
    EH_CP_IDF_GE_5_5
#  define EH_CP_WIFI_GOT_SET_EAP_METHODS         0
#else
#  define EH_CP_WIFI_GOT_SET_EAP_METHODS         1
#endif

/* EAP set domain name: compound IDF version check */
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 4) && \
     ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 4, 0)) || \
    (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 3))
#  define EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME     1
#else
#  define EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME     0
#endif

/* EAP OKC support: same version range as set_domain_name */
#define EH_CP_WIFI_GOT_EAP_OKC                   EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME

/* ═══════════════════════════════════════════════════════════════════════════
 * WiFi DPP (Device Provisioning Protocol)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Supplicant callback DPP: removed from IDF v6.0 */
#if defined(CONFIG_ESP_WIFI_DPP_SUPPORT) && \
    (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0))
#  define EH_CP_WIFI_SUPP_DPP                    1
#else
#  define EH_CP_WIFI_SUPP_DPP                    0
#endif

/* WiFi DPP events: IDF v5.5+ */
#if defined(CONFIG_ESP_WIFI_DPP_SUPPORT) && EH_CP_IDF_GE_5_5
#  define EH_CP_WIFI_DPP                         1
#else
#  define EH_CP_WIFI_DPP                         0
#endif

/* Generic DPP (either path) */
#if EH_CP_WIFI_SUPP_DPP || EH_CP_WIFI_DPP
#  define EH_CP_WIFI_DPP_ANY                     1
#else
#  define EH_CP_WIFI_DPP_ANY                     0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * System
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_ALLOW_FULL_APP_DESC
#  define EH_CP_ALLOW_FULL_APP_DESC              1
#else
#  define EH_CP_ALLOW_FULL_APP_DESC              0
#endif

#if EH_CP_ALLOW_FULL_APP_DESC && (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(5, 3, 1))
#  define EH_CP_GOT_EFUSE_BLK_REV_APP_DESC      1
#else
#  define EH_CP_GOT_EFUSE_BLK_REV_APP_DESC      0
#endif

#if EH_CP_ALLOW_FULL_APP_DESC && EH_CP_IDF_GE_5_4
#  define EH_CP_GOT_MMU_PAGE_SIZE_APP_DESC       1
#else
#  define EH_CP_GOT_MMU_PAGE_SIZE_APP_DESC       0
#endif

#ifdef CONFIG_ESP_HOSTED_USE_MEMPOOL
#  define EH_CP_USE_MEMPOOL                      1
#else
#  define EH_CP_USE_MEMPOOL                      0
#endif

#endif /* ESP_HOSTED_CP_IDF_COMPAT_H */
