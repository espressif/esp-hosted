#ifndef __esp_wpa_utils_h_
#define __esp_wpa_utils_h_

/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

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


#endif
