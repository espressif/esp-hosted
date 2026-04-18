/* Thin wrappers for peer data transfer — maps to custom RPC unserialised API */
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "ctrl_api.h"
#include "eh_host_ext_peer_data_transfer.h"

static void (*s_peer_cb)(uint32_t, const uint8_t *, size_t, void *) = NULL;
static void *s_peer_ctx = NULL;

int eh_host_peer_data_send(uint32_t msg_id_to_send,
        const uint8_t *data_to_send, size_t data_len_to_send)
{
    ctrl_cmd_t *req = (ctrl_cmd_t *)calloc(1, sizeof(ctrl_cmd_t));
    if (!req) return -1;

    req->msg_type = CTRL_REQ;
    req->msg_id = CTRL_REQ_CUSTOM_RPC;
    req->u.custom_rpc_unserialised_data.custom_msg_id = msg_id_to_send;
    req->u.custom_rpc_unserialised_data.data = (uint8_t *)data_to_send;
    req->u.custom_rpc_unserialised_data.data_len = data_len_to_send;

    ctrl_cmd_t *resp = send_custom_rpc_unserialised_req_to_slave(req);
    int ret = (resp && resp->resp_event_status == SUCCESS) ? 0 : -1;

    free(req);
    if (resp) free(resp);
    return ret;
}

int eh_host_peer_data_register_callback(uint32_t msg_id_exp,
        void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                         size_t data_len_recvd, void *local_context),
        void *local_context)
{
    s_peer_cb = callback;
    s_peer_ctx = local_context;
    return 0;
}
