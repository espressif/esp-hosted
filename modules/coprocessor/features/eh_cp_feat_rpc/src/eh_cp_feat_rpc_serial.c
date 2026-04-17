/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_READY
/*
 * eh_cp_feat_rpc_serial.c
 *
 * Serial transport helpers for eh_cp_feat_rpc:
 *
 * Design:
 *   serial_rx  — Registry-1 RX callback for ESP_SERIAL_IF.
 *                Called from recv_task with a fully-reassembled heap buffer
 *                (core owns reassembly; no fixed-size cap).
 *                Passes the buffer directly into process_req() which does
 *                its own malloc+memcpy into the pserial queue.  No secondary
 *                scratch buffer needed here.
 *
 *   serial_read — Called by pserial_task as recv(arg.data, arg.len).
 *                 arg.data already holds the malloc'd copy made in
 *                 protocomm_pserial_data_ready.  Just confirm the length.
 *
 *   serial_write — Fragments + enqueues outgoing frames to host transport.
 */

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "eh_log.h"
#include "eh_transport.h"       /* PRIO_Q_SERIAL, MORE_FRAGMENT */
#include "eh_transport_cp.h"    /* send_to_host_queue */
#include "eh_frame.h"           /* interface_buffer_handle_t */
#include "eh_interface.h"

#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ll.h"

#define ETH_DATA_LEN 1500

static const char TAG[] = "ext_rpc_serial";

/* ── serial_read ─────────────────────────────────────────────────────────── */
/*
 * Called by pserial_task: len = recv(arg.data, arg.len)
 *
 * protocomm_pserial_data_ready() already malloc'd + memcpy'd the incoming
 * bytes into arg.data before queuing the work item.  serial_rx clears
 * s_serial_rx immediately after calling process_req, so by the time
 * pserial_task calls recv(), s_serial_rx may already be stale.
 *
 * arg.data already holds the correct bytes — just confirm the length.
 */
ssize_t eh_cp_feat_rpc_serial_read(uint8_t *data, ssize_t len)
{
    (void)data;   /* already populated by protocomm_pserial_data_ready */
    return len;
}

/* ── serial_write ────────────────────────────────────────────────────────── */

esp_err_t eh_cp_feat_rpc_serial_write(uint8_t *data, ssize_t len)
{
    uint8_t  *pos      = data;
    int32_t   left_len = len;
    int32_t   frag_len = 0;
    static uint16_t seq_num = 0;

    do {
        interface_buffer_handle_t buf_handle = {0};

        seq_num++;
        buf_handle.if_type  = ESP_SERIAL_IF;
        buf_handle.if_num   = 0;
        buf_handle.seq_num  = seq_num;

        if (left_len > ETH_DATA_LEN) {
            frag_len         = ETH_DATA_LEN;
            buf_handle.flags = MORE_FRAGMENT;
        } else {
            frag_len                      = left_len;
            buf_handle.flags              = 0;
            buf_handle.priv_buffer_handle = data;
            buf_handle.free_buf_handle    = free;
        }

        buf_handle.payload     = pos;
        buf_handle.payload_len = frag_len;

        if (send_to_host_queue(&buf_handle, PRIO_Q_SERIAL)) {
            free(data);
            return ESP_FAIL;
        }

        ESP_HEXLOGV("serial_tx", pos, frag_len, 32);

        left_len -= frag_len;
        pos      += frag_len;
    } while (left_len);

    return ESP_OK;
}

/* ── serial_rx (Registry-1 RX callback for ESP_SERIAL_IF) ───────────────── */
/*
 * Called from recv_task with a fully-reassembled heap buffer.  Core handles
 * fragment reassembly with no fixed size cap (dynamic realloc).
 *
 * buf is the core's reassembly buffer — valid only for this call.
 * process_req() → protocomm_pserial_data_ready() does its own malloc+memcpy
 * into the queue, so we pass buf directly with no secondary copy here.
 */
esp_err_t eh_cp_feat_rpc_serial_rx(void *ctx, void *buf,
                                           uint16_t len, void *eb)
{
    (void)ctx;
    (void)eb;

    if (!buf || len == 0) {
        ESP_LOGW(TAG, "serial_rx: empty buffer, drop");
        return ESP_OK;
    }

    ESP_LOGD(TAG, "serial_rx: len=%u", len);
    ESP_HEXLOGV("serial_rx", buf, len, 32);

    return eh_cp_feat_rpc_process_req(buf, len);
}
#endif /* EH_CP_FEAT_RPC_READY */
