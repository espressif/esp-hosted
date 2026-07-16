/* SPDX-License-Identifier: Apache-2.0 */
/* RPC API wrappers for the system feature. */

#ifndef EH_HOST_SYS_H_
#define EH_HOST_SYS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_mac.h"
#include "esp_wifi_types.h"

#include "eh_host_api_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_sys_get_cp_fw_version(eh_host_coprocessor_fwver_t *ver_info);

/* mode = WIFI_IF_STA/AP; mac is 6 bytes. */
esp_err_t eh_host_sys_get_mac(wifi_interface_t mode, uint8_t mac[6]);
esp_err_t eh_host_sys_set_mac(wifi_interface_t mode, const uint8_t mac[6]);

/* Full esp_mac_type_t get/set; mac_len = 6 / 8 / 2 per type. */
esp_err_t eh_host_iface_mac_addr_set(uint8_t *mac, size_t mac_len, esp_mac_type_t type);
esp_err_t eh_host_iface_mac_addr_get(uint8_t *mac, size_t mac_len, esp_mac_type_t type);

/* Returns required mac buffer length, or 0 for unknown type. */
size_t eh_host_iface_mac_addr_len_get(esp_mac_type_t type);

/* App descriptor; wire carries version + idf_target only. */
#define ESP_HOSTED_APP_DESC_MAGIC_WORD (0xABCD5432)

#ifndef EH_HOST_SYS_APP_DESC_T_DEFINED
#define EH_HOST_SYS_APP_DESC_T_DEFINED 1
typedef struct {
    uint32_t magic_word;
    uint32_t secure_version;
    uint32_t reserv1[2];
    char     version[32];
    char     project_name[32];
    char     time[16];
    char     date[16];
    char     idf_ver[32];
    uint8_t  app_elf_sha256[32];
    uint16_t min_efuse_blk_rev_full;
    uint16_t max_efuse_blk_rev_full;
    uint8_t  mmu_page_size;
    uint8_t  reserv3[3];
    uint32_t reserv2[18];
} esp_hosted_app_desc_t;
#endif

esp_err_t eh_host_sys_get_cp_app_desc(esp_hosted_app_desc_t *app_desc);

esp_err_t eh_host_sys_register_event_handlers(void);
esp_err_t eh_host_sys_unregister_event_handlers(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_SYS_H_ */
