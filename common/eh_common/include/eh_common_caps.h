// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef EH_COMMON_CAPS_H
#define EH_COMMON_CAPS_H

/* eh_common_caps.h — wire-contract capability bitmasks (PRIV_EVENT_INIT TLV).
 * Single source of truth; adding a bit is wire-breaking — bump EH_CAPS_VERSION. */

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
#define EH_TRANSPORT_CP_SPI_HD_2_DATA_LINES       (1u << 0)
#define EH_TRANSPORT_CP_SPI_HD_4_DATA_LINES       (1u << 1)
#define EH_TRANSPORT_CP_SPI_HD_1_DATA_LINE        (1u << 2)
#define ESP_WLAN_SUPPORT                          (1u << 4)
#define ESP_EXT_CAP_OT                            (1u << 6)
#define ESP_WLAN_UART_SUPPORT                     (1u << 8)

#define ESP_EXT_CAP_WIFI_ENT                      (1u << 3)
#define ESP_EXT_CAP_WIFI_DPP                      (1u << 7)
#define ESP_EXT_CAP_HOST_PS                       (1u << 10)
#define ESP_EXT_CAP_NW_SPLIT                      (1u << 11)
#define ESP_EXT_CAP_CUSTOM_RPC                    (1u << 12)
#define EH_EXT_CAP_BT_INTERFACE                   (1u << 13)

/* ── Tier 3: feat_caps[8] indices ──────────────────────────────────────── */
/* Do not reuse indices; retired slots stay RSVD. */
#define EH_FEAT_CAPS_COUNT                8u

#define EH_FEAT_IDX_WIFI                  0u
#define EH_FEAT_IDX_BT                    1u
#define EH_FEAT_IDX_OTA                   2u
#define EH_FEAT_IDX_PS                    3u
#define EH_FEAT_IDX_NW_SPLIT              4u
#define EH_FEAT_IDX_CUSTOM_RPC            5u
/* indices 6 and 7 reserved */

/* Wire-format chip ID byte from PRIV_EVENT_INIT TLV. */
#define EH_PRIV_FIRMWARE_CHIP_UNRECOGNIZED  0xFFu
#define EH_PRIV_FIRMWARE_CHIP_ESP32         0x00u
#define EH_PRIV_FIRMWARE_CHIP_ESP32S2       0x02u
#define EH_PRIV_FIRMWARE_CHIP_ESP32C3       0x05u
#define EH_PRIV_FIRMWARE_CHIP_ESP32S3       0x09u
#define EH_PRIV_FIRMWARE_CHIP_ESP32C2       0x0Cu
#define EH_PRIV_FIRMWARE_CHIP_ESP32C6       0x0Du
#define EH_PRIV_FIRMWARE_CHIP_ESP32H2       0x10u
#define EH_PRIV_FIRMWARE_CHIP_ESP32C61      0x14u
#define EH_PRIV_FIRMWARE_CHIP_ESP32C5       0x17u
#define EH_PRIV_FIRMWARE_CHIP_ESP32H4       0x1Cu

/* Bump when any bit above changes. */
#define EH_CAPS_VERSION                   2u

#endif /* EH_COMMON_CAPS_H */
