/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_host_feat_openthread.h"
#include "eh_host_auto_init.h"

esp_err_t eh_host_feat_openthread_init(void)
{
    return ESP_OK;
}

esp_err_t eh_host_feat_openthread_deinit(void)
{
    return ESP_OK;
}

EH_HOST_FEAT_REGISTER(eh_host_feat_openthread_init,
                      eh_host_feat_openthread_deinit,
                      "openthread", 350);
