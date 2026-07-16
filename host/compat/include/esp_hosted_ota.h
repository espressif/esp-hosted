/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat surface for OTA. Two redirect families: legacy
 * esp_hosted_slave_ota_* and modern esp_hosted_cp_ota_*. */

#ifndef EH_COMPAT_ESP_HOSTED_OTA_H_
#define EH_COMPAT_ESP_HOSTED_OTA_H_

#include "esp_err.h"

#include "eh_host_cp_ota.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Legacy slave-ota status tokens; values stable across builds. */
enum {
    ESP_HOSTED_SLAVE_OTA_ACTIVATED,
    ESP_HOSTED_SLAVE_OTA_COMPLETED,
    ESP_HOSTED_SLAVE_OTA_NOT_REQUIRED,
    ESP_HOSTED_SLAVE_OTA_NOT_STARTED,
    ESP_HOSTED_SLAVE_OTA_IN_PROGRESS,
    ESP_HOSTED_SLAVE_OTA_FAILED,
};

esp_err_t esp_hosted_slave_ota(const char *image_url);

#define esp_hosted_slave_ota_begin()           eh_host_cp_ota_begin()
#define esp_hosted_slave_ota_write(b, l)       eh_host_cp_ota_write((const uint8_t *)(b), (l))
#define esp_hosted_slave_ota_end()             eh_host_cp_ota_end()
#define esp_hosted_slave_ota_activate()        eh_host_cp_ota_activate()

#define esp_hosted_cp_ota_begin()              eh_host_cp_ota_begin()
#define esp_hosted_cp_ota_write(b, l)          eh_host_cp_ota_write((const uint8_t *)(b), (l))
#define esp_hosted_cp_ota_end()                eh_host_cp_ota_end()
#define esp_hosted_cp_ota_activate()           eh_host_cp_ota_activate()

#ifdef __cplusplus
}
#endif

#endif /* EH_COMPAT_ESP_HOSTED_OTA_H_ */
