/*
 * SPDX-FileCopyrightText: 2019-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_wifi_netif.h"
#include "rpc_api.h"
#include "os_wrapper.h"
#include "transport_drv.h"
#include "esp_log.h"
#include "stats.h"

static const char TAG[] = "H_W_netif";

//
//  Purpose of this module is provide object oriented abstraction to wifi interfaces
//  in order to integrate wifi as esp-netif driver
//

/**
 * @brief WiFi netif driver structure
 */
struct wifi_netif_driver {
    esp_netif_driver_base_t base;
    wifi_interface_t wifi_if;
};

//static const char* TAG = "wifi_netif";

/**
 * @brief Local storage for netif handles and callbacks for specific wifi interfaces
 */
static esp_netif_receive_t s_wifi_rxcbs[MAX_WIFI_IFS] = { NULL };
static esp_netif_t *s_wifi_netifs[MAX_WIFI_IFS] = { NULL };

/**
 * @brief WiFi netif driver IO functions, a thin glue layer
 *         to the original wifi interface API
 */
static esp_err_t wifi_sta_receive(void *buffer, uint16_t len, void *eb)
{
	ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, len, ESP_LOG_DEBUG);
    return s_wifi_rxcbs[WIFI_IF_STA](s_wifi_netifs[WIFI_IF_STA], buffer, len, eb);
}

#ifdef CONFIG_ESP_WIFI_SOFTAP_SUPPORT
static esp_err_t wifi_ap_receive(void *buffer, uint16_t len, void *eb)
{
	ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, len, ESP_LOG_DEBUG);
    return s_wifi_rxcbs[WIFI_IF_AP](s_wifi_netifs[WIFI_IF_AP], buffer, len, eb);
}
#endif

static void wifi_free(void *h, void* buffer)
{
    if (buffer) {
        //esp_wifi_internal_free_rx_buffer(buffer);
		g_h.funcs->_h_nw_free(buffer);
#if H_MEM_STATS
		h_stats_g.nw_mem_stats.tx_freed++;
#endif
    }
}

static inline esp_err_t l_wifi_transmit(void *h, void *buffer, size_t len)
{
	wifi_netif_driver_t driver = h;
		/* Hosted Tx to Slave */

	#if CONFIG_H_LOWER_MEMCOPY
#if 1
		uint8_t * buf_copy = (uint8_t*)g_h.funcs->_h_nw_calloc(1, len);
		assert(buf_copy);

#if H_MEM_STATS
		h_stats_g.nw_mem_stats.tx_alloc++;
#endif
		/* Keep empty space for ESP payload header, will be filled later */
		g_h.funcs->_h_memcpy(buf_copy+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);
		return esp_hosted_tx(driver->wifi_if, 0, buf_copy, len, H_BUFF_ZEROCOPY, g_h.funcs->_h_nw_free);
#else
		return esp_hosted_tx(driver->wifi_if, 0, buffer, len, H_BUFF_ZEROCOPY, g_h.funcs->_h_nw_free);
#endif
	#else
		uint8_t * buf_copy = (uint8_t*)g_h.funcs->_h_calloc(1, len);
			assert(buf_copy);
			g_h.funcs->_h_memcpy(buf_copy, buffer, len);
			return esp_hosted_tx(driver->wifi_if, 0, buf_copy, len, H_BUFF_NO_ZEROCOPY, H_DEFLT_FREE_FUNC);
	#endif
}

static esp_err_t wifi_transmit(void *h, void *buffer, size_t len)
{
	return l_wifi_transmit(h,buffer,len);
}

static esp_err_t wifi_transmit_wrap(void *h, void *buffer, size_t len, void *netstack_buf)
{
	return l_wifi_transmit(h,buffer,len);
}

static esp_err_t wifi_driver_start(esp_netif_t * esp_netif, void * args)
{
    wifi_netif_driver_t driver = args;
    driver->base.netif = esp_netif;
    esp_netif_driver_ifconfig_t driver_ifconfig = {
            .handle =  driver,
            .transmit = wifi_transmit,
            .transmit_wrap= wifi_transmit_wrap,
            .driver_free_rx_buffer = wifi_free
    };

    return esp_netif_set_driver_config(esp_netif, &driver_ifconfig);
}

void esp_wifi_destroy_if_driver(wifi_netif_driver_t h)
{
    if (h) {
		//TODO: Hosted RPC, just to cleanup rx
        //esp_wifi_internal_reg_rxcb(h->wifi_if, NULL);  // ignore the potential error
                                                       // as the wifi might have been already uninitialized
        s_wifi_netifs[h->wifi_if] = NULL;
    }
    g_h.funcs->_h_free(h);
}

//WIFI_IF_STA -> ESP_STA_IF
//WIFI_IF_AP -> ESP_AP_IF

wifi_netif_driver_t esp_wifi_create_if_driver(wifi_interface_t wifi_if)
{
    wifi_netif_driver_t driver = g_h.funcs->_h_calloc(1, sizeof(struct wifi_netif_driver));
    if (driver == NULL) {
        ESP_LOGE(TAG, "No memory to create a wifi interface handle\n");
        return NULL;
    }
	/* Map WiFi IF to Hosted IFs */
#if 0
	if (wifi_if == ESP_STA_IF)
		driver->wifi_if = WIFI_IF_STA;
	else if (wifi_if == ESP_AP_IF)
		driver->wifi_if = WIFI_IF_AP;
#endif
	/* TODO: ESP_ETH_IF is not compatible with WIFI_IF_ETH */
    driver->wifi_if = wifi_if;
	/* Placeholder for ESP_IF_ETH */
    driver->base.post_attach = wifi_driver_start;
    return driver;
}

esp_err_t esp_wifi_get_if_mac(wifi_netif_driver_t ifx, uint8_t mac[6])
{
    wifi_interface_t wifi_interface = ifx->wifi_if;

    return esp_wifi_get_mac(wifi_interface, mac);
}

bool esp_wifi_is_if_ready_when_started(wifi_netif_driver_t ifx)
{
#ifdef CONFIG_ESP_WIFI_SOFTAP_SUPPORT
    // WiFi rxcb to be register wifi rxcb on start for AP only, station gets it registered on connect event
    return (ifx->wifi_if == WIFI_IF_AP);
#else
    return false;
#endif
}

esp_err_t esp_wifi_register_if_rxcb(wifi_netif_driver_t ifx, esp_netif_receive_t fn, void * arg)
{
    if (ifx->base.netif != arg) {
        ESP_LOGE(TAG, "Invalid argument: supplied netif=%p does not equal to interface netif=%p\n", arg, ifx->base.netif);
        return ESP_ERR_INVALID_ARG;
    }
    wifi_interface_t wifi_interface = ifx->wifi_if;
    s_wifi_rxcbs[wifi_interface] = fn;
    hosted_rxcb_t rxcb = NULL;
    esp_err_t ret;

    switch (wifi_interface)
    {

    case WIFI_IF_STA:
        rxcb = wifi_sta_receive;
        break;

#ifdef CONFIG_ESP_WIFI_SOFTAP_SUPPORT
    case WIFI_IF_AP:
        rxcb = wifi_ap_receive;
        break;
#endif

    default:
        break;
    }

    if (rxcb == NULL) {
        ESP_LOGE(TAG, "Unknown wifi interface id if=%d\n", wifi_interface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Interface must be set before registering Wi-Fi RX callback */
    s_wifi_netifs[wifi_interface] = ifx->base.netif;

    //if ((ret = esp_wifi_internal_reg_rxcb(wifi_interface,  rxcb)) != ESP_OK) {
    if ((ret = esp_hosted_register_wifi_rxcb(wifi_interface,  rxcb)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_internal_reg_rxcb for if=%d failed with %d\n", wifi_interface, ret);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
