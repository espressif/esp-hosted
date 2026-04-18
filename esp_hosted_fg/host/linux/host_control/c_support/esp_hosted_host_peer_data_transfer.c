// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#include <stdlib.h>
#include <string.h>
#include "ctrl_api.h"
#include "esp_hosted_host_ext_peer_data_transfer.h"

#define MAX_PEER_CB 16

typedef struct {
    uint32_t msg_id;
    void (*cb)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
               size_t data_len_recvd, void *local_context);
    void *ctx;
    int in_use;
} peer_cb_t;

static peer_cb_t s_peer_cbs[MAX_PEER_CB];

static int peer_data_dispatch(uint32_t msg_id_recvd,
        const uint8_t *data_recvd, size_t data_len_recvd)
{
    for (int i = 0; i < MAX_PEER_CB; ++i) {
        if (s_peer_cbs[i].in_use && s_peer_cbs[i].msg_id == msg_id_recvd) {
            s_peer_cbs[i].cb(msg_id_recvd, data_recvd, data_len_recvd, s_peer_cbs[i].ctx);
            return SUCCESS;
        }
    }
    return CALLBACK_NOT_REGISTERED;
}

int esp_hosted_register_custom_callback(uint32_t msg_id_exp,
        void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                         size_t data_len_recvd, void *local_context),
        void *local_context)
{
    int i;

    if (callback == NULL) {
        for (i = 0; i < MAX_PEER_CB; ++i) {
            if (s_peer_cbs[i].in_use && s_peer_cbs[i].msg_id == msg_id_exp) {
                s_peer_cbs[i].in_use = 0;
                s_peer_cbs[i].cb = NULL;
                s_peer_cbs[i].ctx = NULL;
                s_peer_cbs[i].msg_id = 0;
                return SUCCESS;
            }
        }
        return CALLBACK_NOT_REGISTERED;
    }

    for (i = 0; i < MAX_PEER_CB; ++i) {
        if (s_peer_cbs[i].in_use && s_peer_cbs[i].msg_id == msg_id_exp) {
            s_peer_cbs[i].cb = callback;
            s_peer_cbs[i].ctx = local_context;
            return SUCCESS;
        }
    }

    for (i = 0; i < MAX_PEER_CB; ++i) {
        if (!s_peer_cbs[i].in_use) {
            s_peer_cbs[i].in_use = 1;
            s_peer_cbs[i].msg_id = msg_id_exp;
            s_peer_cbs[i].cb = callback;
            s_peer_cbs[i].ctx = local_context;
            return SUCCESS;
        }
    }

    return FAILURE;
}

int esp_hosted_peer_data_handle_event(ctrl_cmd_t *app_event)
{
    if (!app_event) {
        return FAILURE;
    }

    if (app_event->msg_id == CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG) {
        custom_rpc_unserialised_data_t *p = &app_event->u.custom_rpc_unserialised_data;
        (void)peer_data_dispatch(p->custom_msg_id, p->data, p->data_len);
    }

    CLEANUP_CTRL_MSG(app_event);
    return SUCCESS;
}

int esp_hosted_send_custom_data(uint32_t msg_id_to_send,
        const uint8_t *data_to_send, size_t data_len_to_send)
{
    ctrl_cmd_t *req = NULL;
    ctrl_cmd_t *resp = NULL;

    if (data_len_to_send && !data_to_send) {
        return FAILURE;
    }

    req = (ctrl_cmd_t *)calloc(1, sizeof(ctrl_cmd_t));
    if (!req) {
        return FAILURE;
    }

    req->msg_id = CTRL_REQ_CUSTOM_RPC_UNSERIALISED_MSG;
    req->u.custom_rpc_unserialised_data.custom_msg_id = msg_id_to_send;
    req->u.custom_rpc_unserialised_data.data = (uint8_t *)data_to_send;
    req->u.custom_rpc_unserialised_data.data_len = data_len_to_send;

    resp = send_custom_rpc_unserialised_req_to_slave(req);
    CLEANUP_CTRL_MSG(req);

    if (!resp || resp->resp_event_status != SUCCESS) {
        if (resp) {
            CLEANUP_CTRL_MSG(resp);
        }
        return FAILURE;
    }

    CLEANUP_CTRL_MSG(resp);
    return SUCCESS;
}
