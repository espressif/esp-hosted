// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __esp_utils_h_
#define __esp_utils_h_

#include "esp.h"

#define WPA_CIPHER_NONE                 BIT(0)
#define WPA_CIPHER_WEP40                BIT(7)
#define WPA_CIPHER_WEP104               BIT(8)
#define WPA_CIPHER_TKIP                 BIT(1)
#define WPA_CIPHER_CCMP                 BIT(3)
#define WPA_CIPHER_AES_128_CMAC         BIT(5)
#define WPA_CIPHER_SMS4                 BIT(10)
#define WPA_CIPHER_GCMP                 BIT(11)
#define WPA_CIPHER_GCMP_256             BIT(12)
#define WPA_CIPHER_BIP_GMAC_128         BIT(13)
#define WPA_CIPHER_BIP_GMAC_256         BIT(14)

enum wpa_alg {
    WIFI_WPA_ALG_NONE   = 0,
    WIFI_WPA_ALG_WEP40  = 1,
    WIFI_WPA_ALG_TKIP   = 2,
    WIFI_WPA_ALG_CCMP   = 3,
    WIFI_WAPI_ALG_SMS4  = 4,
    WIFI_WPA_ALG_WEP104 = 5,
    WIFI_WPA_ALG_WEP    = 6,
    WIFI_WPA_ALG_IGTK   = 7,
    WIFI_WPA_ALG_PMK    = 8,
    WIFI_WPA_ALG_GCMP   = 9,
};

int wpa_cipher_to_alg(int cipher);

char * esp_chipname_from_id(int chipset_id);

#endif
