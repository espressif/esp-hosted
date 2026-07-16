/* SPDX-License-Identifier: Apache-2.0 */
/* Host RPC API umbrella: per-feature wrapper headers conditionally
 * included on the matching EH_HOST_FEAT_* gate. Wrappers are
 * declarations-only; impls live in the active RPC ext. */

#ifndef EH_HOST_H_
#define EH_HOST_H_

#include "eh_host_port_master_config.h"

#include "eh_host_event.h"

#if EH_HOST_FEAT_SYSTEM
#  include "eh_host_sys.h"
#endif

#if EH_HOST_FEAT_WIFI
#  include "eh_host_wifi.h"
#endif

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
#  include "eh_host_wifi_dpp.h"
#endif

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
#  include "eh_host_wifi_ent.h"
#endif

#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
#  include "eh_host_wifi_itwt.h"
#endif

#if EH_HOST_FEAT_BT
#  include "eh_host_bt.h"
#endif

#if EH_HOST_FEAT_OPENTHREAD
#  include "eh_host_openthread.h"
#endif

#if EH_HOST_FEAT_OTA
#  include "eh_host_cp_ota.h"
#endif

#if EH_HOST_FEAT_GPIO_EXP
#  include "eh_host_cp_gpio.h"
#endif

#if EH_HOST_FEAT_CP_EXT_COEX
#  include "eh_host_cp_ext_coex.h"
#endif

#if EH_HOST_FEAT_PEER_DATA
#  include "eh_host_peer_data.h"
#endif

#if EH_HOST_FEAT_HEARTBEAT
#  include "eh_host_heartbeat.h"
#endif

#if EH_HOST_FEAT_NW_SPLIT
#  include "eh_host_nw_split.h"
#endif

#if EH_HOST_FEAT_MEM_MONITOR
#  include "eh_host_mem_monitor.h"
#endif

#endif /* EH_HOST_H_ */
