// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2025 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

/**
 * @file app_peer_data_transfer.c
 * @brief Host-side peer data transfer demo — animal-sound theme
 *
 * The host sends animal sounds (dog/cat/bird) to the CP via
 * eh_host_peer_data_send() with a specific msg_id.  The CP echoes
 * back a translated reply on PEER_MSG_ID_CP_REPLY which is dispatched
 * here through the per-msg-id callback registered with
 * eh_host_peer_data_register_callback().
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <time.h>
#include "test.h"
#include "ctrl_api.h"
#include "app_peer_data_transfer.h"

static const char *TAG = "app_peer_data_xfer";

/* ------------------------------------------------------------------ */
/* Per-animal contexts (static, alive for the process lifetime)        */
/* ------------------------------------------------------------------ */

static animal_ctx_t s_dog_ctx  = { .animal_name = "Dog",  .send_count = 0, .recv_count = 0 };
static animal_ctx_t s_cat_ctx  = { .animal_name = "Cat",  .send_count = 0, .recv_count = 0 };
static animal_ctx_t s_bird_ctx = { .animal_name = "Bird", .send_count = 0, .recv_count = 0 };

/* ------------------------------------------------------------------ */
/* CP-reply callback — registered once, dispatches by context          */
/* ------------------------------------------------------------------ */

/**
 * Called whenever the CP sends PEER_MSG_ID_CP_REPLY.
 * local_context points to the animal_ctx_t that issued the last send,
 * updated atomically before the send so it always matches.
 */
static void on_cp_reply(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                        size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    printf("[%s][%s] recv #%" PRIu32 " — msg_id=0x%" PRIx32
           " len=%zu payload='%.*s'\n",
           TAG, ctx->animal_name, ctx->recv_count,
           msg_id_recvd, data_len_recvd,
           (int)data_len_recvd, (const char *)data_recvd);
}

/* ------------------------------------------------------------------ */
/* peer_data_transfer_event_handler                                    */
/* ------------------------------------------------------------------ */

/**
 * Entry point called by the ctrl event loop for
 * CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG.  Dispatches to the
 * appropriate per-msg-id callback if one is registered.
 */
int peer_data_transfer_event_handler(ctrl_cmd_t *app_event)
{
    if (test_validate_ctrl_event(app_event)) {
        printf("[%s] invalid event[%u]\n", TAG, app_event->msg_id);
        CLEANUP_CTRL_MSG(app_event);
        return FAILURE;
    }

    if (app_event->msg_id == CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG) {
        custom_rpc_unserialised_data_t *p = &app_event->u.custom_rpc_unserialised_data;
        uint32_t mid = p->custom_msg_id;

        /* Locate a registered callback for this msg_id */
        /* We keep it simple: check the three fixed IDs we registered. */
        /* A production impl would use a hash/table from the reusable API. */
        printf("[%s] Received peer msg_id=0x%" PRIx32 " len=%u\n",
               TAG, mid, p->data_len);

        /* Dispatch: call registered callback via the reusable API shim.
         * For the demo we replicate the dispatch inline to keep zero extra
         * state. The reusable API header documents the generic path. */
        if (mid == PEER_MSG_ID_CP_REPLY) {
            /* Route to whichever animal last sent — stored in s_last_ctx */
            /* We use a simple last-sender tracker set by each demo. */
            extern animal_ctx_t *g_last_sender_ctx;
            if (g_last_sender_ctx) {
                on_cp_reply(mid, p->data, p->data_len, g_last_sender_ctx);
            } else {
                printf("[%s] CP reply received but no sender context set\n", TAG);
            }
        } else {
            printf("[%s] Unhandled peer msg_id=0x%" PRIx32 "\n", TAG, mid);
        }
    } else {
        printf("[%s] Unhandled base event: %u\n", TAG, app_event->msg_id);
    }

    CLEANUP_CTRL_MSG(app_event);
    return SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Last-sender tracking (set before every send so the reply callback   */
/* knows which animal context to update)                               */
/* ------------------------------------------------------------------ */

animal_ctx_t *g_last_sender_ctx = NULL;

/* ------------------------------------------------------------------ */
/* Helper: send + wait for reply                                       */
/* ------------------------------------------------------------------ */

static int send_animal_sound(animal_ctx_t *ctx, uint32_t msg_id,
                              const char *sound_str, int wait_sec)
{
    /* Register the CP-reply callback pointing to this animal's context */
    if (eh_host_peer_data_register_callback(PEER_MSG_ID_CP_REPLY,
                on_cp_reply, ctx) != SUCCESS) {
        printf("[%s] Failed to register CP-reply callback\n", TAG);
        return FAILURE;
    }

    g_last_sender_ctx = ctx;
    ctx->send_count++;

    printf("[%s][%s] send #%" PRIu32 " — msg_id=0x%" PRIx32 " payload='%s'\n",
           TAG, ctx->animal_name, ctx->send_count, msg_id, sound_str);

    int ret = eh_host_peer_data_send(msg_id,
            (const uint8_t *)sound_str, strlen(sound_str));
    if (ret != SUCCESS) {
        printf("[%s] send failed (ret=%d)\n", TAG, ret);
        return FAILURE;
    }

    /* Wait for the CP reply (event handler updates recv_count) */
    uint32_t prev_recv = ctx->recv_count;
    time_t deadline = time(NULL) + wait_sec;
    while (ctx->recv_count == prev_recv && time(NULL) < deadline) {
        sleep(1);
        printf("[%s] Waiting for CP reply...\n", TAG);
    }

    if (ctx->recv_count == prev_recv) {
        printf("[%s][%s] Timeout waiting for CP reply\n", TAG, ctx->animal_name);
        return FAILURE;
    }

    printf("[%s][%s] Round-trip complete (sends=%" PRIu32 " recvs=%" PRIu32 ")\n",
           TAG, ctx->animal_name, ctx->send_count, ctx->recv_count);
    return SUCCESS;
}

/* ------------------------------------------------------------------ */
/* Demo entry points                                                   */
/* ------------------------------------------------------------------ */

int peer_data_transfer_demo1_dog_sound(void)
{
    printf("[%s] === Demo 1: Dog sound ===\n", TAG);
    return send_animal_sound(&s_dog_ctx, PEER_MSG_ID_DOG_SOUND, "Woof!", 5);
}

int peer_data_transfer_demo2_cat_sound(void)
{
    printf("[%s] === Demo 2: Cat sound ===\n", TAG);
    return send_animal_sound(&s_cat_ctx, PEER_MSG_ID_CAT_SOUND, "Meow!", 5);
}

int peer_data_transfer_demo3_bird_sound(void)
{
    printf("[%s] === Demo 3: Bird sound ===\n", TAG);
    return send_animal_sound(&s_bird_ctx, PEER_MSG_ID_BIRD_SOUND, "Tweet!", 5);
}
