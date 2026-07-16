/* SPDX-License-Identifier: Apache-2.0 */

#include "esp_err.h"

esp_err_t __real_esp_event_loop_create_default(void);
esp_err_t __real_esp_netif_init(void);

esp_err_t __wrap_esp_event_loop_create_default(void)
{
    esp_err_t rc = __real_esp_event_loop_create_default();
    return (rc == ESP_ERR_INVALID_STATE) ? ESP_OK : rc;
}

esp_err_t __wrap_esp_netif_init(void)
{
    esp_err_t rc = __real_esp_netif_init();
    return (rc == ESP_ERR_INVALID_STATE) ? ESP_OK : rc;
}
