/* SPDX-License-Identifier: Apache-2.0 */

#include "sdkconfig.h"
#include "esp_err.h"

esp_err_t __real_esp_event_loop_create_default(void);
esp_err_t __wrap_esp_event_loop_create_default(void);

esp_err_t __wrap_esp_event_loop_create_default(void)
{
    esp_err_t rc = __real_esp_event_loop_create_default();
    return (rc == ESP_ERR_INVALID_STATE) ? ESP_OK : rc;
}


#if CONFIG_ESP_WIFI_ENABLED
#include "esp_netif.h"

esp_err_t __real_esp_netif_init(void);
esp_netif_t *__real_esp_netif_create_default_wifi_sta(void);
esp_netif_t *__real_esp_netif_create_default_wifi_ap(void);
esp_err_t __wrap_esp_netif_init(void);
esp_netif_t *__wrap_esp_netif_create_default_wifi_sta(void);
esp_netif_t *__wrap_esp_netif_create_default_wifi_ap(void);

esp_err_t __wrap_esp_netif_init(void)
{
    esp_err_t rc = __real_esp_netif_init();
    return (rc == ESP_ERR_INVALID_STATE) ? ESP_OK : rc;
}

esp_netif_t *__wrap_esp_netif_create_default_wifi_sta(void)
{
    esp_netif_t *existing = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    return existing ? existing : __real_esp_netif_create_default_wifi_sta();
}

esp_netif_t *__wrap_esp_netif_create_default_wifi_ap(void)
{
    esp_netif_t *existing = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    return existing ? existing : __real_esp_netif_create_default_wifi_ap();
}
#endif /* CONFIG_ESP_WIFI_ENABLED */
