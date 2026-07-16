/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi_types.h"

/* Band mode for a dual-band CP (used only under CONFIG_SLAVE_SOC_WIFI_SUPPORT_5G).
 * Maps the EH_EXAMPLE_WIFI_BAND Kconfig choice → esp_wifi enum; default 2G (the
 * reliable choice for a 2.4 GHz AP — AUTO can stall on the C5). */
#if defined(CONFIG_EH_EXAMPLE_WIFI_BAND_5G)
#define EH_EXAMPLE_WIFI_BAND_MODE WIFI_BAND_MODE_5G_ONLY
#elif defined(CONFIG_EH_EXAMPLE_WIFI_BAND_AUTO)
#define EH_EXAMPLE_WIFI_BAND_MODE WIFI_BAND_MODE_AUTO
#else
#define EH_EXAMPLE_WIFI_BAND_MODE WIFI_BAND_MODE_2G_ONLY
#endif

/* Optional fixed STA channel width (throughput characterization: 20 vs 40 MHz).
 * Maps the EH_EXAMPLE_WIFI_BANDWIDTH Kconfig choice → esp_wifi enum. AUTO leaves
 * EH_EXAMPLE_WIFI_BW undefined so the width is negotiated with the AP (default). */
#if defined(CONFIG_EH_EXAMPLE_WIFI_BW_HT40)
#define EH_EXAMPLE_WIFI_BW WIFI_BW40
#elif defined(CONFIG_EH_EXAMPLE_WIFI_BW_HT20)
#define EH_EXAMPLE_WIFI_BW WIFI_BW20
#endif

/* Optional STA PHY-protocol bitmap (esp_wifi_set_protocol), band-scoped: 2.4 GHz
 * legacy is 11b/11g, 5 GHz legacy is 11a — advertising 2.4 GHz-only bits on the
 * 5 GHz band makes the CP reject the whole bitmap (ESP_ERR_INVALID_ARG). 11n(HT)
 * applies to both bands, 11ac(VHT) is 5 GHz-only, 11ax(HE) applies to both.
 * DEFAULT leaves EH_EXAMPLE_WIFI_PROTO undefined so the CP's max stands. 11ac/11ax
 * are #ifdef-guarded for older host headers. */
#if defined(CONFIG_EH_EXAMPLE_WIFI_BAND_5G)
#define EH_EXAMPLE_WIFI_PROTO_BASE WIFI_PROTOCOL_11A
#else
#define EH_EXAMPLE_WIFI_PROTO_BASE (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G)
#endif

#if defined(CONFIG_EH_EXAMPLE_WIFI_BAND_5G) && defined(WIFI_PROTOCOL_11AC)
#define EH_EXAMPLE_WIFI_PROTO_VHT WIFI_PROTOCOL_11AC
#else
#define EH_EXAMPLE_WIFI_PROTO_VHT 0
#endif

#ifdef WIFI_PROTOCOL_11AX
#define EH_EXAMPLE_WIFI_PROTO_HE WIFI_PROTOCOL_11AX
#else
#define EH_EXAMPLE_WIFI_PROTO_HE 0
#endif

#if defined(CONFIG_EH_EXAMPLE_WIFI_PROTO_11AX)
#define EH_EXAMPLE_WIFI_PROTO (EH_EXAMPLE_WIFI_PROTO_BASE | WIFI_PROTOCOL_11N | \
                               EH_EXAMPLE_WIFI_PROTO_VHT | EH_EXAMPLE_WIFI_PROTO_HE)
#elif defined(CONFIG_EH_EXAMPLE_WIFI_PROTO_11N)
#define EH_EXAMPLE_WIFI_PROTO (EH_EXAMPLE_WIFI_PROTO_BASE | WIFI_PROTOCOL_11N)
#elif defined(CONFIG_EH_EXAMPLE_WIFI_PROTO_LEGACY)
#define EH_EXAMPLE_WIFI_PROTO (EH_EXAMPLE_WIFI_PROTO_BASE)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* One-time NVS + netif + default event loop + esp_wifi_init.
 * Idempotent — second call is a no-op. */
esp_err_t eh_example_init(void);
esp_err_t eh_example_deinit(void);

/* STA — explicit credentials.  Blocks until IP or MAXIMUM_RETRY.
 * Auth picked from password length (empty → OPEN, else WPA2-PSK).
 * Band auto-selected: works on 2.4 GHz and 5 GHz APs. */
esp_err_t eh_example_sta_connect(const char *ssid, const char *password);
esp_err_t eh_example_sta_disconnect(void);

/* STA — menuconfig credentials (CONFIG_EH_EXAMPLE_WIFI_SSID/PASSWORD). */
esp_err_t eh_example_connect(void);
esp_err_t eh_example_disconnect(void);

/* Active scan.  channel=0 scans all (2.4 GHz + 5 GHz if chip supports).
 * Caller allocates out_records of length *count; *count is in/out
 * (max on entry, actual on return). */
esp_err_t eh_example_scan(uint8_t channel,
                          wifi_ap_record_t *out_records,
                          uint16_t *count);

/* SoftAP.  channel: 1-13 (2.4 GHz) or 36+ (5 GHz, chip-dependent).
 * Auth = OPEN if password is NULL/empty, else WPA2-PSK. */
esp_err_t eh_example_softap_start(const char *ssid,
                                  const char *password,
                                  uint8_t channel);
esp_err_t eh_example_softap_stop(void);

#ifdef __cplusplus
}
#endif
