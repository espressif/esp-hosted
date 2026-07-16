/* SPDX-License-Identifier: Apache-2.0 */
/* Real (non-inline) compat symbols from esp_hosted.h; others are header
 * inlines that forward to eh_host_*. */

#include "esp_hosted.h"

#include "esp_err.h"
#include "sdkconfig.h"
#include "eh_host_port_master_config.h"
#include "eh_host_core.h"
#include "eh_host_feat_system.h"

int esp_hosted_init(void)
{
    return eh_host_init(NULL);
}

int esp_hosted_deinit(void)
{
    return eh_host_deinit();
}

int esp_hosted_connect_to_slave(void)
{
    return eh_host_connect_to_slave();
}

int esp_hosted_get_coprocessor_fwversion(esp_hosted_coprocessor_fwver_t *ver_info)
{
    return eh_host_sys_get_cp_fw_version(ver_info);
}

int esp_hosted_get_cp_info(uint32_t *cp_chip_id, char *cp_target_name,
                           size_t cp_target_name_len)
{
    return eh_host_sys_get_cp_info(cp_chip_id, cp_target_name, cp_target_name_len);
}

/* HTTP fetch is application-owned; caller uses eh_host_cp_ota_* directly. */
esp_err_t esp_hosted_slave_ota(const char *image_url)
{
    (void)image_url;
    return ESP_ERR_NOT_SUPPORTED;
}

