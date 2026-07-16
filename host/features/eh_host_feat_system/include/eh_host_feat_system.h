/* SPDX-License-Identifier: Apache-2.0 */
/* System feature public API: CP version, MAC get/set, heartbeat. */

#ifndef EH_HOST_FEAT_SYSTEM_H_
#define EH_HOST_FEAT_SYSTEM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi_types.h"
#include "eh_host_api_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_feat_system_init(void);
esp_err_t eh_host_feat_system_deinit(void);

esp_err_t eh_host_sys_get_cp_fw_version(eh_host_coprocessor_fwver_t *ver_info);

/* CP chip_id + idf_target; NUL-terminated, truncated if too long. */
esp_err_t eh_host_sys_get_cp_info(uint32_t *chip_id,
                                  char *target_name, size_t target_name_len);

esp_err_t eh_host_sys_get_mac(wifi_interface_t mode, uint8_t mac[6]);
esp_err_t eh_host_sys_set_mac(wifi_interface_t mode, const uint8_t mac[6]);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_SYSTEM_H_ */
