/* SPDX-License-Identifier: Apache-2.0 */
/* msg_id-keyed dispatcher with lazy esp_event subscription. */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "eh_host_event.h"
#include "eh_host_peer_data.h"
#include "eh_host_feat_peer_data.h"
#include "eh_host_port_sync.h"
#include "eh_host_port.h"

static const char *TAG = "eh_peer_dispatch";

typedef struct {
    uint32_t                 msg_id;
    eh_host_peer_data_cb_t   cb;
    void                    *ctx;
} entry_t;

static eh_host_port_mutex_t *s_lock;
static entry_t         *s_entries;
static size_t           s_count;
static size_t           s_capacity;
static bool             s_handler_registered;

static void dispatcher(void *handler_arg, esp_event_base_t base,
                       int32_t event_id, void *event_data)
{
    (void)handler_arg; (void)base; (void)event_id;
    if (!event_data) return;
    const eh_host_event_peer_data_t *evt = event_data;

    /* Snapshot under lock; cb may re-enter register/send. */
    eh_host_peer_data_cb_t  cb  = NULL;
    void                   *ctx = NULL;
    if (s_lock) {
        eh_host_port_mutex_lock(s_lock);
        for (size_t i = 0; i < s_count; i++) {
            if (s_entries[i].msg_id == evt->msg_id) {
                cb  = s_entries[i].cb;
                ctx = s_entries[i].ctx;
                break;
            }
        }
        if (!cb) {
            ESP_LOGW(TAG, "evt: no cb for msg_id=%" PRIu32 " (registered=%zu)",
                     evt->msg_id, s_count);
            for (size_t i = 0; i < s_count; i++) {
                ESP_LOGW(TAG, "evt: reg[%zu]=%" PRIu32 " cb=%p ctx=%p",
                         i, s_entries[i].msg_id, (void *)s_entries[i].cb, s_entries[i].ctx);
            }
        }
        eh_host_port_mutex_unlock(s_lock);
    }
    if (cb) {
        cb(evt->msg_id, evt->data, evt->len, ctx);
    }

    if (evt->data) {
        free((void *)evt->data);
    }
}

esp_err_t eh_host_peer_data_register(uint32_t msg_id,
                                     eh_host_peer_data_cb_t cb,
                                     void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_lock) {
        s_lock = eh_host_port_mutex_create();
        if (!s_lock) return ESP_ERR_NO_MEM;
    }

    eh_host_port_mutex_lock(s_lock);

    for (size_t i = 0; i < s_count; i++) {
        if (s_entries[i].msg_id == msg_id) {
            s_entries[i].cb  = cb;
            s_entries[i].ctx = ctx;
            ESP_LOGI(TAG, "register update: msg_id=%" PRIu32 " cb=%p ctx=%p",
                     msg_id, (void *)cb, ctx);
            eh_host_port_mutex_unlock(s_lock);
            return ESP_OK;
        }
    }

    if (s_count == s_capacity) {
        size_t new_cap = s_capacity ? s_capacity * 2 : 4;
        entry_t *grown = realloc(s_entries, new_cap * sizeof(*grown));
        if (!grown) {
            eh_host_port_mutex_unlock(s_lock);
            return ESP_ERR_NO_MEM;
        }
        s_entries  = grown;
        s_capacity = new_cap;
    }
    s_entries[s_count].msg_id = msg_id;
    s_entries[s_count].cb     = cb;
    s_entries[s_count].ctx    = ctx;
    s_count++;
    ESP_LOGI(TAG, "register add: msg_id=%" PRIu32 " cb=%p ctx=%p count=%zu",
             msg_id, (void *)cb, ctx, s_count);

    esp_err_t reg_rc = ESP_OK;
    if (!s_handler_registered) {
        reg_rc = esp_event_handler_register(EH_HOST_EVENT,
                                            EH_HOST_EVENT_PEER_DATA_RX,
                                            &dispatcher, NULL);
        if (reg_rc == ESP_OK) {
            s_handler_registered = true;
            ESP_LOGI(TAG, "event handler registered for EH_HOST_EVENT_PEER_DATA_RX");
        } else {
            ESP_LOGE(TAG, "event handler register failed rc=%d", (int)reg_rc);
        }
    }
    eh_host_port_mutex_unlock(s_lock);
    return reg_rc;
}
