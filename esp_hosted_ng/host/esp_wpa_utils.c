// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include "esp_wpa_utils.h"


int wpa_cipher_to_alg(int cipher)
{
	switch (cipher) {
	case WLAN_CIPHER_SUITE_CCMP:
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
	case WLAN_CIPHER_SUITE_AES_CMAC:
		return WIFI_WPA_ALG_IGTK;
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
