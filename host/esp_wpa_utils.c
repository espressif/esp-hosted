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

#include "esp_wpa_utils.h"


int wpa_cipher_to_alg(int cipher)
{
    switch (cipher) {
    case WLAN_CIPHER_SUITE_CCMP :
        return WIFI_WPA_ALG_CCMP;
#ifdef CONFIG_GCMP
    case WLAN_CIPHER_SUITE_GCMP_256:
    case WLAN_CIPHER_SUITE_GCMP:
        return WIFI_WPA_ALG_GCMP;
#endif
    case WLAN_CIPHER_SUITE_TKIP:
        return WIFI_WPA_ALG_TKIP;
    case WLAN_CIPHER_SUITE_WEP104:
        return WIFI_WPA_ALG_WEP104;
    case WLAN_CIPHER_SUITE_WEP40:
        return WIFI_WPA_ALG_WEP40;
    }
    return WIFI_WPA_ALG_NONE;
}

int wpa_cipher_key_len(int cipher)
{
    switch (cipher) {
    case WLAN_CIPHER_SUITE_TKIP:
#ifdef CONFIG_GCMP
    case WLAN_CIPHER_GCMP_SUITE_256:
#endif
#ifdef CONFIG_GMAC
    case WLAN_CIPHER_BIP_GMAC_SUITE_256:
#endif
        return 32;
    case WLAN_CIPHER_SUITE_CCMP:
#ifdef CONFIG_GCMP
    case WLAN_CIPHER_SUITE_GCMP:
#endif
#ifdef CONFIG_GMAC
    case WLAN_CIPHER_SUITE_BIP_GMAC_128:
#endif
#if 0
    case WLAN_CIPHER_SUITE_AES_128_CMAC:
        return 16;
#endif
    case WLAN_CIPHER_SUITE_WEP104:
        return 13;
    case WLAN_CIPHER_SUITE_WEP40:
        return 5;
    }

    return 0;
}
