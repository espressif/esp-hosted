/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file app_main.c
 * @brief Peer Data Transfer example — Linux FG coprocessor side
 *
 * Demonstrates the eh_cp_feat_peer_data_transfer extension:
 *  - Register per-msg-id callbacks with an animal context (local_context)
 *  - Send peer data to the host
 *
 * Animal theme: the CP listens for the host sending animal sounds and
 * responds by echoing back a translated reply, keyed on msg_id.
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "eh_cp.h"
#include "eh_cp_feat_peer_data.h"

static const char *TAG = "peer_data_transfer_fg";

/* ------------------------------------------------------------------ */
/* Animal context — passed as local_context to each callback           */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *animal_name;
    uint32_t    recv_count;
} animal_ctx_t;

static animal_ctx_t s_cat_ctx   = { .animal_name = "Cat",   .recv_count = 0 };
static animal_ctx_t s_dog_ctx   = { .animal_name = "Dog",   .recv_count = 0 };
static animal_ctx_t s_human_ctx = { .animal_name = "Human", .recv_count = 0 };

/* ------------------------------------------------------------------ */
/* Message IDs — agreed upon between host and coprocessor              */
/* ------------------------------------------------------------------ */

/* Match MCU host example message IDs */
#define MSG_ID_CAT      1   /* Request: small data */
#define MSG_ID_MEOW     2   /* Response: echo small data */
#define MSG_ID_DOG      3   /* Request: medium data */
#define MSG_ID_WOOF     4   /* Response: echo medium data */
#define MSG_ID_HUMAN    5   /* Request: large data */
#define MSG_ID_HELLO    6   /* Response: echo large data */

/* ------------------------------------------------------------------ */
/* Per-msg-id callbacks                                                */
/* ------------------------------------------------------------------ */

static void on_cat(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                   size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    ESP_LOGI(TAG, "[%s] recv #%" PRIu32 " — msg_id=0x%" PRIx32 " len=%zu",
             ctx->animal_name, ctx->recv_count,
             msg_id_recvd, data_len_recvd,
             (int)data_len_recvd);
    eh_cp_feat_peer_data_send(MSG_ID_MEOW,
            data_recvd, data_len_recvd);
}

static void on_dog(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                   size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    ESP_LOGI(TAG, "[%s] recv #%" PRIu32 " — msg_id=0x%" PRIx32 " len=%zu",
             ctx->animal_name, ctx->recv_count,
             msg_id_recvd, data_len_recvd,
             (int)data_len_recvd);
    eh_cp_feat_peer_data_send(MSG_ID_WOOF,
            data_recvd, data_len_recvd);
}

static void on_human(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                     size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    ESP_LOGI(TAG, "[%s] recv #%" PRIu32 " — msg_id=0x%" PRIx32 " len=%zu",
             ctx->animal_name, ctx->recv_count,
             msg_id_recvd, data_len_recvd,
             (int)data_len_recvd);
    eh_cp_feat_peer_data_send(MSG_ID_HELLO,
            data_recvd, data_len_recvd);
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    esp_event_loop_create_default();
    ESP_LOGI(TAG, "Peer Data Transfer Example — Linux FG CP");

    /* Init core CP */
    eh_cp_init();

    /* Register per-msg-id callbacks, each with its own animal context */
    ESP_ERROR_CHECK(eh_cp_feat_peer_data_register_callback(
            MSG_ID_CAT, on_cat, &s_cat_ctx));
    ESP_ERROR_CHECK(eh_cp_feat_peer_data_register_callback(
            MSG_ID_DOG, on_dog, &s_dog_ctx));
    ESP_ERROR_CHECK(eh_cp_feat_peer_data_register_callback(
            MSG_ID_HUMAN, on_human, &s_human_ctx));

    ESP_LOGI(TAG, "Peer data transfer extension ready — waiting for host messages");

    /* No startup payload; data is sent only as event echoes */
}
