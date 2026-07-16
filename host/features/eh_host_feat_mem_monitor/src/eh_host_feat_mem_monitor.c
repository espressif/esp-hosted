/* SPDX-License-Identifier: Apache-2.0 */
/* Mem-monitor lifecycle: init/deinit register Event_MemMonitor handler. */

#include "esp_err.h"

#include "eh_host_port_master_config.h"
#include "eh_host_feat_mem_monitor.h"
#include "eh_host_auto_init.h"
#include "eh_host_mem_monitor.h"
#include "eh_host_port.h"

#if EH_HOST_FEAT_MEM_MONITOR_READY

#define MM_TAG "eh_mem_monitor"

int eh_host_feat_mem_monitor_init(void)
{
    ESP_LOGI(MM_TAG, "mem_monitor feature init");
    if (eh_host_mem_monitor_register_event_handlers() != ESP_OK) {
        ESP_LOGE(MM_TAG, "mem_monitor: event handler register failed");
        return -1;
    }
    return 0;
}

int eh_host_feat_mem_monitor_deinit(void)
{
    ESP_LOGI(MM_TAG, "mem_monitor feature deinit");
    eh_host_mem_monitor_unregister_event_handlers();
    return 0;
}

#if EH_HOST_FEAT_MEM_MONITOR_AUTO_INIT
EH_HOST_FEAT_REGISTER(eh_host_feat_mem_monitor_init,
                      eh_host_feat_mem_monitor_deinit,
                      "mem_monitor", 400);
#endif

#else /* !EH_HOST_FEAT_MEM_MONITOR_READY */

int eh_host_feat_mem_monitor_init(void)   { return 0; }
int eh_host_feat_mem_monitor_deinit(void) { return 0; }

#endif /* EH_HOST_FEAT_MEM_MONITOR_READY */
