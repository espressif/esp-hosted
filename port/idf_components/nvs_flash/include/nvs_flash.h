/* SPDX-License-Identifier: Apache-2.0 */
/*
 * nvs_flash.h — IDF NVS API for non-IDF hosts.  Bodies are no-op
 * (no flash partition on Linux); impl lives in
 * port/esp_idf_port/nvs_flash/src/nvs_flash.c.
 */

#ifndef EH_COMPAT_NVS_FLASH_H_
#define EH_COMPAT_NVS_FLASH_H_

#include "esp_err.h"

#ifndef ESP_ERR_NVS_BASE
#  define ESP_ERR_NVS_BASE                    0x1100
#endif
#define ESP_ERR_NVS_NO_FREE_PAGES             (ESP_ERR_NVS_BASE + 0x0d)
#define ESP_ERR_NVS_NEW_VERSION_FOUND         (ESP_ERR_NVS_BASE + 0x14)

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_COMPAT_NVS_FLASH_H_ */
