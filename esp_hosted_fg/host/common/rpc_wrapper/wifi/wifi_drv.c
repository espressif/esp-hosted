// Copyright 2015-2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "esp_wifi.h"
//TODO: remove this header, move declarations to better header
#include "rpc_wrapper.h"

/* TODO: move hosted specific functions and headers to diff file */
#include "transport_drv.h"
#include "serial_drv.h"

#include "esp_log.h"
#include "stats.h"

DEFINE_LOG_TAG(wifi_drv);
//#include "netdev_api.h"

//ESP_EVENT_DEFINE_BASE(WIFI_EVENT);
//struct hosted_config_t g_h = HOSTED_CONFIG_INIT_DEFAULT();
semaphore_handle_t esp_hosted_up_sem;

/* Set additional WiFi features and capabilities */
uint64_t g_wifi_feature_caps =
#if CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE
    CONFIG_FEATURE_WPA3_SAE_BIT |
#endif
#if CONFIG_SPIRAM
    CONFIG_FEATURE_CACHE_TX_BUF_BIT |
#endif
#if CONFIG_ESP_WIFI_FTM_INITIATOR_SUPPORT
    CONFIG_FEATURE_FTM_INITIATOR_BIT |
#endif
#if CONFIG_ESP_WIFI_FTM_RESPONDER_SUPPORT
    CONFIG_FEATURE_FTM_RESPONDER_BIT |
#endif
0;


//static void reset_slave(void)
//{
//	g_h.funcs->_h_config_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, H_GPIO_MODE_DEF_OUTPUT);
//
//	g_h.funcs->_h_write_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, 1);
//	g_h.funcs->_h_blocking_delay(100);
//	g_h.funcs->_h_write_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, 0);
//	g_h.funcs->_h_blocking_delay(100);
//	g_h.funcs->_h_write_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, 1);
//
//	/* stop spi transactions short time to avoid slave sync issues */
//	g_h.funcs->_h_blocking_delay(100000);
//}

//static void transport_driver_event_handler(uint8_t event)
//{
//	switch(event)
//	{
//		case TRANSPORT_ACTIVE:
//		{
//			g_h.funcs->_h_post_semaphore_from_isr(esp_hosted_up_sem);
//			/* Initiate rpc now */
//#if CONFIG_TRANSPORT_LOG_LEVEL
//			printf("Base transport is set-up\n\r");
//#endif
//			//rpc_init(rpc_event_handler);
//			break;
//		}
//		default:
//		break;
//	}
//}

//static esp_err_t hosted_init(void)
//{
//    esp_hosted_up_sem = g_h.funcs->_h_create_binary_semaphore();
//	if (!esp_hosted_up_sem)
//		printf("Unable to create sem %s:%u\n", __func__, __LINE__);
//
//	reset_slave();
//	transport_init(transport_driver_event_handler);
//	if (init_rpc_lib()) {
//		printf("init rpc lib failed\n");
//		return ESP_FAIL;
//	}
//	register_event_callbacks();
//    return ESP_OK;
//}
//static esp_err_t hosted_deinit(void)
//{
//	unregister_event_callbacks();
//	/* Call rpc library init */
//	rpc_platform_deinit();
//	deinit_rpc_lib();
//    return ESP_OK;
//}

//esp_err_t esp_netif_init(void)
//{
//    return ESP_OK;
//}
//
//esp_err_t esp_netif_deinit(void)
//{
//    return ESP_OK;
//}

void esp_hosted_up_cb(void)
{
	if (esp_hosted_up_sem)
		g_h.funcs->_h_post_semaphore_from_isr(esp_hosted_up_sem);
}

esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
	ESP_LOGV(TAG, "Trigger Wifi init");
    esp_hosted_up_sem = g_h.funcs->_h_create_semaphore(1);
	if (!esp_hosted_up_sem)
		printf("Unable to create sem %s:%u\n", __func__, __LINE__);
	g_h.funcs->_h_get_semaphore(esp_hosted_up_sem, portMAX_DELAY);

	H_ERROR_CHECK(esp_hosted_init(esp_hosted_up_cb));
	g_h.funcs->_h_get_semaphore(esp_hosted_up_sem, portMAX_DELAY);

#if 0//TEST_RAW_TP
	/* Just block the app main so that wifi will be stopped and only raw tp will be running */
	g_h.funcs->_h_get_semaphore(esp_hosted_up_sem, portMAX_DELAY);
#endif
	/*g_h.funcs->_h_get_semaphore(esp_hosted_up_sem, portMAX_DELAY); //Wait until slave is ready*/
	return test_wifi_init(config);
}

esp_err_t esp_wifi_deinit(void)
{
	return test_wifi_deinit();
}

esp_err_t esp_wifi_set_mode(int mode)
{
	return test_wifi_set_mode(mode);
}

esp_err_t esp_wifi_get_mode(wifi_mode_t *mode)
{
	return test_wifi_get_mode(mode);
}

esp_err_t esp_wifi_start(void)
{
	return test_wifi_start();
}

esp_err_t esp_wifi_stop(void)
{
	return test_wifi_stop();
}

esp_err_t esp_wifi_disconnect(void)
{
	return test_wifi_disconnect();
}

esp_err_t esp_wifi_connect(void)
{
	return test_wifi_connect();
}

esp_err_t esp_wifi_set_config(int interface, wifi_config_t *conf)
{
	return test_wifi_set_config(interface, conf);
}

esp_err_t esp_wifi_get_config(int interface, wifi_config_t *conf)
{
	return test_wifi_get_config(interface, conf);
}

esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6])
{
	return test_wifi_get_mac_addr(ifx, mac);
}

esp_err_t esp_wifi_set_mac(wifi_interface_t ifx, uint8_t mac[6])
{
	return test_wifi_set_mac_addr(ifx, mac);
}

esp_err_t esp_wifi_scan_start(wifi_scan_config_t *config, bool block)
{
	return test_wifi_scan_start(config, block);
}

esp_err_t esp_wifi_scan_stop(void)
{
	return test_wifi_scan_stop();
}

esp_err_t esp_wifi_scan_get_ap_num(uint16_t *number)
{
	return test_wifi_scan_get_ap_num(number);
}

esp_err_t esp_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records)
{
	return test_wifi_scan_get_ap_records(number, ap_records);
}

esp_err_t esp_wifi_clear_ap_list(void)
{
	return test_wifi_clear_ap_list();
}

esp_err_t esp_wifi_restore(void)
{
	return test_wifi_restore();
}

esp_err_t esp_wifi_clear_fast_connect(void)
{
	return test_wifi_clear_fast_connect();
}

esp_err_t esp_wifi_deauth_sta(uint16_t aid)
{
	return test_wifi_deauth_sta(aid);
}

esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info)
{
	return test_wifi_sta_get_ap_info(ap_info);
}

esp_err_t esp_wifi_set_ps(wifi_ps_type_t type)
{
	return test_wifi_set_ps(type);
}

esp_err_t esp_wifi_get_ps(wifi_ps_type_t *type)
{
	return test_wifi_get_ps(type);
}

esp_err_t esp_wifi_set_storage(wifi_storage_t storage)
{
	return test_wifi_set_storage(storage);
}

esp_err_t esp_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw)
{
	return test_wifi_set_bandwidth(ifx, bw);
}

esp_err_t esp_wifi_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw)
{
	return test_wifi_get_bandwidth(ifx, bw);
}

esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second)
{
	return test_wifi_set_channel(primary, second);
}

esp_err_t esp_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second)
{
	return test_wifi_get_channel(primary, second);
}

esp_err_t esp_wifi_set_country_code(const char *country, bool ieee80211d_enabled)
{
	return test_wifi_set_country_code(country, ieee80211d_enabled);
}

esp_err_t esp_wifi_get_country_code(char *country)
{
	return test_wifi_get_country_code(country);
}

esp_err_t esp_wifi_set_country(const wifi_country_t *country)
{
	return test_wifi_set_country(country);
}

esp_err_t esp_wifi_get_country(wifi_country_t *country)
{
	return test_wifi_get_country(country);
}

#if 0
esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap)
{
	return test_wifi_set_protocol(ifx, protocol_bitmap);
}

esp_err_t esp_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap)
{
	return test_wifi_set_protocol(ifx, protocol_bitmap);
}
#endif
