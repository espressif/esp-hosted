// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef EH_COMMON_CAPS_H
#define EH_COMMON_CAPS_H

/*
 * eh_common_caps.h — wire-contract capability bitmasks
 *
 * These definitions appear in the PRIV_EVENT_INIT TLV handshake and
 * must be identical on both the slave (ESP coprocessor) and the host
 * (Linux kernel module, MCU host component, etc.).
 *
 * This is the SINGLE SOURCE OF TRUTH for all capability bit values.
 * All code (transport, features, host) includes this header.
 * eh_caps.h (transport) adds TLV tags and legacy structs on top.
 *
 * Two tiers:
 *   caps     (uint8_t)        — Transport-level capability word (V1 legacy).
 *   feat_caps[8] (uint32_t[8])— Feature bitmap array sent via PRIV TLV at boot.
 *                               Each feature sets bits via eh_cp_add_feat_cap_bits_idx().
 *
 * IMPORTANT: Adding a new bit is a wire-breaking change.
 * Bump EH_CAPS_VERSION when doing so and update the CI MD5 check.
 */

#include "eh_common_interface.h"

/* ── Tier 1: caps (uint8_t) ─────────────────────────────────────────────── */
#define ESP_WLAN_SDIO_SUPPORT                     (1u << 0)
#define ESP_BT_UART_SUPPORT                       (1u << 1)
#define ESP_BT_SDIO_SUPPORT                       (1u << 2)
#define ESP_BLE_ONLY_SUPPORT                      (1u << 3)
#define ESP_BR_EDR_ONLY_SUPPORT                   (1u << 4)
#define ESP_WLAN_SPI_SUPPORT                      (1u << 5)
#define ESP_BT_SPI_SUPPORT                        (1u << 6)
#define ESP_CHECKSUM_ENABLED                      (1u << 7)

/* ── Tier 2: ext_caps (uint32_t) ────────────────────────────────────────── */
#define ESP_WLAN_SUPPORT                          (1u << 0)
#define ESP_WLAN_UART_SUPPORT                     (1u << 1)
#define EH_TRANSPORT_CP_SPI_HD_4_DATA_LINES (1u << 2)
#define EH_TRANSPORT_CP_SPI_HD_2_DATA_LINES (1u << 3)
#define ESP_EXT_CAP_WIFI_ENT                      (1u << 4)
#define ESP_EXT_CAP_WIFI_DPP                      (1u << 5)
#define ESP_EXT_CAP_HOST_PS                       (1u << 6)
#define ESP_EXT_CAP_NW_SPLIT                      (1u << 7)
#define ESP_EXT_CAP_CUSTOM_RPC                    (1u << 8)

/* ── Tier 3: feat_caps[8] indices ──────────────────────────────────────── */
/*
 * feat_caps is an array of 8 uint32_t words, each word is an independent
 * bitmask. Extensions use eh_cp_add_feat_cap_bits_idx(index, mask)
 * to set bits at boot time; the accumulated array is sent in the PRIV TLV.
 *
 * Index assignments (do not reuse; mark retired slots as RSVD):
 *   [0]  WiFi feature flags
 *   [1]  BT/BLE feature flags
 *   [2]  OTA feature flags
 *   [3]  Power save feature flags
 *   [4]  Network split feature flags
 *   [5]  Custom RPC feature flags
 *   [6]  Reserved
 *   [7]  Reserved
 */
#define EH_FEAT_CAPS_COUNT                8u

#define EH_FEAT_IDX_WIFI                  0u
#define EH_FEAT_IDX_BT                    1u
#define EH_FEAT_IDX_OTA                   2u
#define EH_FEAT_IDX_PS                    3u
#define EH_FEAT_IDX_NW_SPLIT              4u
#define EH_FEAT_IDX_CUSTOM_RPC            5u
/* indices 6 and 7 reserved */

/* ── Caps version ───────────────────────────────────────────────────────── */
/* Increment when any bit definition above changes */
#define EH_CAPS_VERSION                   1u

#endif /* EH_COMMON_CAPS_H */
