/* SPDX-License-Identifier: Apache-2.0 */
/* WiFi feature lifecycle; owns sub-extension lifecycle (Ent/iTWT/DPP). */

#include <stdbool.h>

#include "esp_err.h"
#include "eh_host_port_master_config.h"
#include "eh_host_feat_wifi.h"
#include "eh_host_feat_wifi_priv.h"
#include "eh_host_auto_init.h"
#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_port.h"

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
int eh_host_feat_wifi_ext_ent_init(void);
int eh_host_feat_wifi_ext_ent_deinit(void);
#endif
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
int eh_host_feat_wifi_ext_itwt_init(void);
int eh_host_feat_wifi_ext_itwt_deinit(void);
#endif
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
int eh_host_feat_wifi_ext_dpp_init(void);
int eh_host_feat_wifi_ext_dpp_deinit(void);
#endif

#define WIFI_TAG "eh_wifi"

int eh_host_feat_wifi_init(void)
{
    int rc = eh_host_feat_rpc_ext_v2_register_wifi_event_handlers();
    if (rc != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "event handler register failed (%d)", rc);
        return -1;
    }

    /* enterprise → iTWT → DPP (matches CP order). */
#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
    eh_host_feat_wifi_ext_ent_init();
#endif
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
    eh_host_feat_wifi_ext_itwt_init();
#endif
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    eh_host_feat_wifi_ext_dpp_init();
#endif
    return 0;
}

int eh_host_feat_wifi_deinit(void)
{
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    eh_host_feat_wifi_ext_dpp_deinit();
#endif
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
    eh_host_feat_wifi_ext_itwt_deinit();
#endif
#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
    eh_host_feat_wifi_ext_ent_deinit();
#endif
    return eh_host_feat_rpc_ext_v2_unregister_wifi_event_handlers();
}

EH_HOST_FEAT_REGISTER(eh_host_feat_wifi_init, eh_host_feat_wifi_deinit, "wifi", 150);
