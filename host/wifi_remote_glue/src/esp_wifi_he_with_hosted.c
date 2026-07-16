/* SPDX-License-Identifier: Apache-2.0 */

#if __has_include(<esp_wifi_remote.h>)

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi_he.h"

static const char *TAG = "esp_wifi_he_glue";

#define UNSUPPORTED_RET()                                                      \
    do {                                                                       \
        ESP_LOGW(TAG, "%s: not wired (RPC pending)", __func__);                \
        return ESP_ERR_NOT_SUPPORTED;                                          \
    } while (0)

const char eh_wifi_he_with_hosted_anchor = 0;

esp_err_t esp_wifi_sta_btwt_setup(wifi_btwt_setup_config_t *config)
{
    (void)config;
    UNSUPPORTED_RET();
}

esp_err_t esp_wifi_sta_btwt_teardown(uint8_t btwt_id)
{
    (void)btwt_id;
    UNSUPPORTED_RET();
}

esp_err_t esp_wifi_sta_get_btwt_num(uint8_t *btwt_number)
{
    (void)btwt_number;
    UNSUPPORTED_RET();
}

esp_err_t esp_wifi_sta_btwt_get_info(uint8_t btwt_number,
                                     esp_wifi_btwt_info_t *btwt_info)
{
    (void)btwt_number;
    (void)btwt_info;
    UNSUPPORTED_RET();
}

esp_err_t esp_wifi_enable_rx_statistics(bool rx_stats, bool rx_mu_stats)
{
    (void)rx_stats;
    (void)rx_mu_stats;
    UNSUPPORTED_RET();
}

esp_err_t esp_wifi_enable_tx_statistics(esp_wifi_aci_t aci, bool tx_stats)
{
    (void)aci;
    (void)tx_stats;
    UNSUPPORTED_RET();
}

esp_err_t esp_wifi_enable_bsscolor_collision_detection(wifi_interface_t ifx,
                                                       bool enable)
{
    (void)ifx;
    (void)enable;
    UNSUPPORTED_RET();
}

#endif /* __has_include(<esp_wifi_remote.h>) */
