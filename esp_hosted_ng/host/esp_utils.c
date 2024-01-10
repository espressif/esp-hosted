// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include "esp_utils.h"


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

char * esp_chipname_from_id(int chipset_id)
{
	if (chipset_id == ESP_FIRMWARE_CHIP_ESP32)
		return "ESP32";
	if (chipset_id == ESP_FIRMWARE_CHIP_ESP32S2)
		return "ESP32-S2";
	if (chipset_id == ESP_FIRMWARE_CHIP_ESP32S3)
		return "ESP32-S3";
	if (chipset_id == ESP_FIRMWARE_CHIP_ESP32C2)
		return "ESP32-C2";
	if (chipset_id == ESP_FIRMWARE_CHIP_ESP32C3)
		return "ESP32-C3";
	if (chipset_id == ESP_FIRMWARE_CHIP_ESP32C6)
		return "ESP32-C6";

	return "Unknown Chip";
}
