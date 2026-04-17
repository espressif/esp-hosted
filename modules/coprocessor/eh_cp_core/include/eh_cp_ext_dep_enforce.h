/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * eh_cp_ext_dep_enforce.h
 *
 * Compile-time enforcement of inter-extension dependencies.
 * Include at the end of eh_cp_master_config.h so all
 * EH_CP_* macros are already defined.
 *
 * Rule: if extension A requires extension B, add a #error here.
 * Kconfig `depends on` handles menuconfig visibility; this catches
 * manual sdkconfig edits or stale caches.
 */

#ifndef ESP_HOSTED_CP_EXT_DEP_ENFORCE_H
#define ESP_HOSTED_CP_EXT_DEP_ENFORCE_H

#if EH_CP_FEAT_NW_SPLIT_READY && !EH_CP_FEAT_WIFI_READY
#  error "EH_CP_FEAT_NW_SPLIT_READY requires EH_CP_FEAT_WIFI_READY"
#endif

#if EH_CP_FEAT_WIFI_EXT_ENT_READY && !EH_CP_FEAT_WIFI_READY
#  error "EH_CP_FEAT_WIFI_EXT_ENT_READY requires EH_CP_FEAT_WIFI_READY"
#endif

#if EH_CP_FEAT_WIFI_EXT_ITWT_READY && !EH_CP_FEAT_WIFI_READY
#  error "EH_CP_FEAT_WIFI_EXT_ITWT_READY requires EH_CP_FEAT_WIFI_READY"
#endif

#if EH_CP_FEAT_WIFI_EXT_DPP_READY && !EH_CP_FEAT_WIFI_READY
#  error "EH_CP_FEAT_WIFI_EXT_DPP_READY requires EH_CP_FEAT_WIFI_READY"
#endif

#endif /* ESP_HOSTED_CP_EXT_DEP_ENFORCE_H */
