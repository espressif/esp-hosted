/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EH_CP_EVENTS_H__
#define __EH_CP_EVENTS_H__

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EH_CP_TIMEOUT_IN_MSEC(x)          pdMS_TO_TICKS(x)
#define ESP_HOSTED_CP_TIMEOUT_IN_SEC(x)           pdMS_TO_TICKS(x*1000)

/* Event base for ESP Hosted events (Declaration only) */
ESP_EVENT_DECLARE_BASE(EH_CP_EVENT); /* Definition at higher layer */

#define EH_CP_EVT_DFLT_TIMEOUT       (400)
#define EH_CP_EVT_LONG_TIMEOUT       (2000)

/* ESP Hosted event types (core-only; feature events live in extensions) */
typedef enum {
    EH_CP_EVT_INVALID = 0,                            /* Invalid event */

    /* System events */
    EH_CP_EVT_ESP_INIT = 1,                           /* ESP init event */
    EH_CP_EVT_PRIVATE_RPC_READY = 2,                  /* PRIV TLVs parsed; RPC endpoints ready */

    EH_CP_EVT_MAX = 3                                 /* Maximum event ID */
} eh_cp_event_t;

/* Data posted with EH_CP_EVT_PRIVATE_RPC_READY */
typedef struct {
    char req_ep[16];
    char evt_ep[16];
} eh_cp_rpc_ep_config_t;


#ifdef __cplusplus
}
#endif

#endif /* __EH_CP_EVENTS_H__ */
