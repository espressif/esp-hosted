// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "esp_wifi_remote.h"
#include "esp_hosted_api.h"
#include "esp_check.h"
#include "transport_drv.h"
#include "rpc_wrap.h"
#include "esp_log.h"

/** Macros **/
static const char *TAG="H_API";

/** Exported variables **/
struct esp_remote_channel {
	transport_channel_t *t_chan;
};

//static semaphore_handle_t transport_up_sem;

typedef esp_err_t (*esp_remote_channel_rx_fn_t)(void *h, void *buffer,
		void *buff_to_free, size_t len);
typedef esp_err_t (*esp_remote_channel_tx_fn_t)(void *h, void *buffer, size_t len);

/** Inline functions **/

/** Exported Functions **/
static void transport_active_cb(void)
{
	//g_h.funcs->_h_post_semaphore(transport_up_sem);
}

#if 0
static void create_esp_hosted_transport_up_sem(void)
{
	if (!transport_up_sem) {
		transport_up_sem = g_h.funcs->_h_create_semaphore(1);
		assert(transport_up_sem);
		/* clear semaphore */
		g_h.funcs->_h_get_semaphore(transport_up_sem, 0);
	}
}

esp_err_t esp_hosted_setup(void)
{
	create_esp_hosted_transport_up_sem();
	g_h.funcs->_h_get_semaphore(transport_up_sem, portMAX_DELAY);
	g_h.funcs->_h_post_semaphore(transport_up_sem);
	return ESP_OK;
}
#endif

esp_err_t esp_hosted_init(void(*esp_hosted_up_cb)(void))
{
	//create_esp_hosted_transport_up_sem();
	ESP_LOGI(TAG, "ESP-Hosted starting. Hosted_Tasks: prio:%u, stack: %u RPC_task_stack: %u",
			DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, RPC_TASK_STACK_SIZE);
	ESP_ERROR_CHECK(transport_drv_init(transport_active_cb));
	ESP_ERROR_CHECK(rpc_init());
	rpc_register_event_callbacks();

	if (esp_hosted_up_cb)
		esp_hosted_up_cb();
	return ESP_OK;
}

esp_err_t esp_hosted_deinit(void)
{
	ESP_LOGI(TAG, "ESP-Hosted deinit\n");
	rpc_unregister_event_callbacks();
	ESP_ERROR_CHECK(rpc_deinit());
	ESP_ERROR_CHECK(transport_drv_deinit());
	return ESP_OK;
}

esp_err_t esp_hosted_reinit(void(*esp_hosted_up_cb)(void))
{
	ESP_LOGI(TAG, "ESP-Hosted re-init\n");
	ESP_ERROR_CHECK(esp_hosted_deinit());
	ESP_ERROR_CHECK(esp_hosted_init(esp_hosted_up_cb));
	ESP_ERROR_CHECK(transport_drv_reconfigure());
	return ESP_OK;
}

esp_err_t esp_hosted_slave_reset(void)
{
	ESP_LOGI(TAG, "ESP-Hosted Try to communicate with ESP-Hosted slave\n");
	return transport_drv_reconfigure();
}


esp_remote_channel_t esp_hosted_add_channel(esp_remote_channel_config_t config,
		esp_remote_channel_tx_fn_t *tx, const esp_remote_channel_rx_fn_t rx)
{
	transport_channel_t *t_chan = NULL;
	esp_remote_channel_t eh_chan = NULL;

	eh_chan = g_h.funcs->_h_calloc(sizeof(struct esp_remote_channel), 1);
	assert(eh_chan);

	t_chan = transport_drv_add_channel(eh_chan, config->if_type, config->secure, tx, rx);
	if (t_chan) {
		*tx = t_chan->tx;
		eh_chan->t_chan = t_chan;
		return eh_chan;
	} else {
		g_h.funcs->_h_free(eh_chan);
	}

	return NULL;
}


esp_err_t esp_hosted_remove_channel(esp_remote_channel_t eh_chan)
{
	if (eh_chan && eh_chan->t_chan) {
		transport_drv_remove_channel(eh_chan->t_chan);
		g_h.funcs->_h_free(eh_chan);
		return ESP_OK;
	}

	return ESP_FAIL;
}

esp_err_t esp_hosted_wifi_init(const wifi_init_config_t *arg)
{
	ESP_ERROR_CHECK(transport_drv_reconfigure());
	return rpc_wifi_init(arg);
}

esp_err_t esp_hosted_wifi_deinit(void)
{
	return rpc_wifi_deinit();
}

esp_err_t esp_hosted_wifi_set_mode(wifi_mode_t mode)
{
	return rpc_wifi_set_mode(mode);
}

esp_err_t esp_hosted_wifi_get_mode(wifi_mode_t* mode)
{
	return rpc_wifi_get_mode(mode);
}

esp_err_t esp_hosted_wifi_start(void)
{
	return rpc_wifi_start();
}

esp_err_t esp_hosted_wifi_stop(void)
{
	return rpc_wifi_stop();
}

esp_err_t esp_hosted_wifi_connect(void)
{
	ESP_LOGI(TAG, "%s",__func__);
	return rpc_wifi_connect();
}

esp_err_t esp_hosted_wifi_disconnect(void)
{
	return rpc_wifi_disconnect();
}

esp_err_t esp_hosted_wifi_set_config(wifi_interface_t interface, wifi_config_t *conf)
{
	return rpc_wifi_set_config(interface, conf);
}

esp_err_t esp_hosted_wifi_get_config(wifi_interface_t interface, wifi_config_t *conf)
{
	return rpc_wifi_get_config(interface, conf);
}

esp_err_t esp_hosted_wifi_get_mac(wifi_interface_t mode, uint8_t *out_mac)
{
	return rpc_wifi_get_mac(mode, out_mac);
}

esp_err_t esp_hosted_wifi_set_mac(wifi_interface_t mode, const uint8_t *mac)
{
	return rpc_wifi_set_mac(mode, mac);
}

esp_err_t esp_hosted_wifi_scan_start(const wifi_scan_config_t *config, bool block)
{
	return rpc_wifi_scan_start(config, block);
}

esp_err_t esp_hosted_wifi_scan_stop(void)
{
	return rpc_wifi_scan_stop();
}

esp_err_t esp_hosted_wifi_scan_get_ap_num(uint16_t *number)
{
	return rpc_wifi_scan_get_ap_num(number);
}

esp_err_t esp_hosted_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records)
{
	return rpc_wifi_scan_get_ap_records(number, ap_records);
}

esp_err_t esp_hosted_wifi_clear_ap_list(void)
{
	return rpc_wifi_clear_ap_list();
}

esp_err_t esp_hosted_wifi_restore(void)
{
	return rpc_wifi_restore();
}

esp_err_t esp_hosted_wifi_clear_fast_connect(void)
{
	return rpc_wifi_clear_fast_connect();
}

esp_err_t esp_hosted_wifi_deauth_sta(uint16_t aid)
{
	return rpc_wifi_deauth_sta(aid);
}

esp_err_t esp_hosted_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info)
{
	return rpc_wifi_sta_get_ap_info(ap_info);
}

esp_err_t esp_hosted_wifi_set_ps(wifi_ps_type_t type)
{
	return rpc_wifi_set_ps(type);
}

esp_err_t esp_hosted_wifi_get_ps(wifi_ps_type_t *type)
{
	return rpc_wifi_get_ps(type);
}

esp_err_t esp_hosted_wifi_set_storage(wifi_storage_t storage)
{
	return rpc_wifi_set_storage(storage);
}

esp_err_t esp_hosted_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw)
{
	return rpc_wifi_set_bandwidth(ifx, bw);
}

esp_err_t esp_hosted_wifi_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw)
{
	return rpc_wifi_get_bandwidth(ifx, bw);
}

esp_err_t esp_hosted_wifi_set_channel(uint8_t primary, wifi_second_chan_t second)
{
	return rpc_wifi_set_channel(primary, second);
}

esp_err_t esp_hosted_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second)
{
	return rpc_wifi_get_channel(primary, second);
}

esp_err_t esp_hosted_wifi_set_country_code(const char *country, bool ieee80211d_enabled)
{
	return rpc_wifi_set_country_code(country, ieee80211d_enabled);
}

esp_err_t esp_hosted_wifi_get_country_code(char *country)
{
	return rpc_wifi_get_country_code(country);
}

esp_err_t esp_hosted_wifi_set_country(const wifi_country_t *country)
{
	return rpc_wifi_set_country(country);
}

esp_err_t esp_hosted_wifi_get_country(wifi_country_t *country)
{
	return rpc_wifi_get_country(country);
}

esp_err_t esp_hosted_wifi_ap_get_sta_list(wifi_sta_list_t *sta)
{
	return rpc_wifi_ap_get_sta_list(sta);
}

esp_err_t esp_hosted_wifi_ap_get_sta_aid(const uint8_t mac[6], uint16_t *aid)
{
	return rpc_wifi_ap_get_sta_aid(mac, aid);
}

esp_err_t esp_hosted_wifi_sta_get_rssi(int *rssi)
{
	return rpc_wifi_sta_get_rssi(rssi);
}

esp_err_t esp_hosted_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap)
{
    return rpc_wifi_set_protocol(ifx, protocol_bitmap);
}

esp_err_t esp_hosted_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap)
{
    return rpc_wifi_get_protocol(ifx, protocol_bitmap);
}


#ifdef __cplusplus
}
#endif
