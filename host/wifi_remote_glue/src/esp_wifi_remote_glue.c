/* SPDX-License-Identifier: Apache-2.0 */
/* esp_wifi_remote_* strong defs forwarding to eh_host_wifi_* / eh_host_sys_*. */

#include <stdint.h>
#include <stdlib.h>

#include "esp_err.h"

#if __has_include(<esp_wifi_remote.h>)

#include "esp_wifi.h"
#include "esp_wifi_types.h"

#include "eh_host_sys.h"
#include "eh_host_wifi.h"
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
#include "eh_host_wifi_itwt.h"
#endif

#include "esp_wifi_remote.h"

#if EH_HOST_TYPE_MCU && EH_HOST_FEAT_WIFI_READY
#include "eh_common_interface.h"
#include "esp_hosted_wifi_remote_glue.h"

static esp_remote_channel_t s_sta_ch;
static esp_remote_channel_t s_ap_ch;

/* Drop RX for an iface that isn't admitting (would reach a half-torn-down netif),
 * else deliver. free() on drop matches esp_wifi_remote_channel_rx's no-handler free. */
static esp_err_t hosted_channel_rx_guard(void *h, void *buffer,
                                         void *buff_to_free, size_t len)
{
    bool is_ap = (h == (void *)s_ap_ch);
    if (__builtin_expect(eh_host_wifi_rx_admitted(is_ap), 1)) {
        return esp_wifi_remote_channel_rx(h, buffer, buff_to_free, len);
    }
    free(buff_to_free);
    return ESP_OK;
}
#endif

const char eh_wifi_remote_glue_anchor = 0;

esp_err_t esp_wifi_remote_init(const wifi_init_config_t *config)
{
#if EH_HOST_TYPE_MCU && EH_HOST_FEAT_WIFI_READY
    struct esp_remote_channel_config sta_cfg = { .if_type = ESP_STA_IF, .secure = false };
    struct esp_remote_channel_config ap_cfg  = { .if_type = ESP_AP_IF,  .secure = false };
    esp_remote_channel_tx_fn_t tx_fn = NULL;

    s_sta_ch = esp_hosted_add_channel(&sta_cfg, &tx_fn, hosted_channel_rx_guard);
    if (s_sta_ch) esp_wifi_remote_channel_set(WIFI_IF_STA, s_sta_ch, tx_fn);

    s_ap_ch = esp_hosted_add_channel(&ap_cfg, &tx_fn, hosted_channel_rx_guard);
    if (s_ap_ch) esp_wifi_remote_channel_set(WIFI_IF_AP, s_ap_ch, tx_fn);
#endif
    return eh_host_wifi_init(config);
}

esp_err_t esp_wifi_remote_deinit(void)
{
    esp_err_t rc = eh_host_wifi_deinit();
#if EH_HOST_TYPE_MCU && EH_HOST_FEAT_WIFI_READY
    if (s_sta_ch) { esp_hosted_remove_channel(s_sta_ch); s_sta_ch = NULL; }
    if (s_ap_ch)  { esp_hosted_remove_channel(s_ap_ch);  s_ap_ch  = NULL; }
#endif
    return rc;
}

esp_err_t esp_wifi_remote_start(void)
{
    return eh_host_wifi_start();
}

esp_err_t esp_wifi_remote_stop(void)
{
    return eh_host_wifi_stop();
}

esp_err_t esp_wifi_remote_connect(void)
{
    return eh_host_wifi_connect();
}

esp_err_t esp_wifi_remote_disconnect(void)
{
    return eh_host_wifi_disconnect();
}

esp_err_t esp_wifi_remote_set_mode(wifi_mode_t mode)
{
    return eh_host_wifi_set_mode(mode);
}

esp_err_t esp_wifi_remote_get_mode(wifi_mode_t *mode)
{
    return eh_host_wifi_get_mode(mode);
}

esp_err_t esp_wifi_remote_set_config(wifi_interface_t interface,
                                     wifi_config_t *conf)
{
    return eh_host_wifi_set_config(interface, conf);
}

esp_err_t esp_wifi_remote_get_config(wifi_interface_t interface,
                                     wifi_config_t *conf)
{
    return eh_host_wifi_get_config(interface, conf);
}

/* CP owns its NVS; host accepts WIFI_STORAGE_* silently. */
esp_err_t esp_wifi_remote_set_storage(wifi_storage_t storage)
{
    (void)storage;
    return ESP_OK;
}

/* MAC lives in eh_host_sys, not eh_host_wifi. */
esp_err_t esp_wifi_remote_get_mac(wifi_interface_t ifx, uint8_t mac[6])
{
    return eh_host_sys_get_mac(ifx, mac);
}

esp_err_t esp_wifi_remote_set_mac(wifi_interface_t ifx, const uint8_t mac[6])
{
    return eh_host_sys_set_mac(ifx, mac);
}

#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
#include "esp_wifi_types_generic.h"

esp_err_t esp_wifi_sta_itwt_setup(wifi_itwt_setup_config_t *setup_config)
{ return eh_host_wifi_itwt_setup(setup_config); }

esp_err_t esp_wifi_sta_itwt_teardown(int flow_id)
{ return eh_host_wifi_itwt_teardown(flow_id); }

esp_err_t esp_wifi_sta_itwt_suspend(int flow_id, int suspend_time_ms)
{ return eh_host_wifi_itwt_suspend(flow_id, suspend_time_ms); }

esp_err_t esp_wifi_sta_itwt_get_flow_id_status(int *flow_id_bitmap)
{ return eh_host_wifi_itwt_get_flow_id_status(flow_id_bitmap); }

esp_err_t esp_wifi_sta_itwt_send_probe_req(int timeout_ms)
{ return eh_host_wifi_itwt_send_probe_req(timeout_ms); }

esp_err_t esp_wifi_sta_itwt_set_target_wake_time_offset(int offset_us)
{ return eh_host_wifi_itwt_set_target_wake_time_offset(offset_us); }

esp_err_t esp_wifi_sta_twt_config(wifi_twt_config_t *config)
{ return eh_host_wifi_sta_twt_config(config); }
#endif /* EH_HOST_FEAT_WIFI_EXT_ITWT_READY */

#endif /* __has_include(<esp_wifi_remote.h>) */
