/* SPDX-License-Identifier: Apache-2.0 */
/* EH_HOST_EVENT bus — base, unified ID enum, lifecycle payloads. */

#ifndef EH_HOST_EVENT_H_
#define EH_HOST_EVENT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_system.h"   /* esp_reset_reason_t */

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(EH_HOST_EVENT);

/* Unified ID enum — single source so features sharing EH_HOST_EVENT don't collide. */
enum {
    /* ── Core / system lifecycle ──────────────────────────────── */
    EH_HOST_EVENT_CP_INIT             = 0,  /* posted from system feature on CP boot */
    EH_HOST_EVENT_CP_HEARTBEAT        = 1,  /* posted from heartbeat feature */
    EH_HOST_EVENT_TRANSPORT_FAILURE   = 2,  /* posted from eh_host_core/transport_events.c */
    EH_HOST_EVENT_TRANSPORT_UP        = 3,  /* posted from eh_host_core/transport_events.c */
    EH_HOST_EVENT_TRANSPORT_DOWN      = 4,  /* posted from eh_host_core/transport_events.c */

    /* ── Feature events (posted by the feature; payload structs in
     *    the feature's header — see comments) ─────────────────── */
    EH_HOST_EVENT_MEM_MONITOR         = 5,  /* feat_mem_monitor; payload = eh_host_event_mem_info_t (eh_host_misc_types.h) */
    EH_HOST_EVENT_PEER_DATA_RX        = 6,  /* feat_peer_data; payload = eh_host_event_peer_data_t (eh_host_peer_data.h) */
    EH_HOST_EVENT_NW_SPLIT_STATUS     = 7,  /* feat_nw_split; payload = eh_host_nw_split_status_t (eh_host_nw_split.h) */
    EH_HOST_EVENT_GPIO_EXP_INT        = 8,  /* feat_gpio_exp; payload TBD when wire carries the event */
};

/* ── Lifecycle payloads (CP_INIT / CP_HEARTBEAT) ───────────────── */

typedef struct {
    esp_reset_reason_t reason;
} eh_host_event_init_t;

typedef struct {
    uint32_t heartbeat;
} eh_host_event_heartbeat_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_EVENT_H_ */
