/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file app_main.c
 * @brief Peer Data Transfer example — MCU coprocessor side
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
#include "esp_log.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "eh_cp.h"
#include "eh_cp_feat_peer_data.h"

static const char *TAG = "peer_data_transfer_mcu";

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

#define MSG_ID_CAT      1   /* host → CP: small data  */
#define MSG_ID_MEOW     2   /* CP → host: echo small  */
#define MSG_ID_DOG      3   /* host → CP: medium data */
#define MSG_ID_WOOF     4   /* CP → host: echo medium */
#define MSG_ID_HUMAN    5   /* host → CP: large data  */
#define MSG_ID_HELLO    6   /* CP → host: echo large  */

/* ------------------------------------------------------------------ */
/* Per-msg-id callbacks                                                */
/* ------------------------------------------------------------------ */

static void on_cat(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                   size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    ESP_LOGI(TAG, "[%s] recv #%" PRIu32 " len=%zu",
             ctx->animal_name, ctx->recv_count, data_len_recvd);
    eh_cp_feat_peer_data_send(MSG_ID_MEOW, data_recvd, data_len_recvd);
}

static void on_dog(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                   size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    ESP_LOGI(TAG, "[%s] recv #%" PRIu32 " len=%zu",
             ctx->animal_name, ctx->recv_count, data_len_recvd);
    eh_cp_feat_peer_data_send(MSG_ID_WOOF, data_recvd, data_len_recvd);
}

static void on_human(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                     size_t data_len_recvd, void *local_context)
{
    animal_ctx_t *ctx = (animal_ctx_t *)local_context;
    ctx->recv_count++;
    ESP_LOGI(TAG, "[%s] recv #%" PRIu32 " len=%zu",
             ctx->animal_name, ctx->recv_count, data_len_recvd);
    eh_cp_feat_peer_data_send(MSG_ID_HELLO, data_recvd, data_len_recvd);
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    esp_event_loop_create_default();
    ESP_LOGI(TAG, "Peer Data Transfer Example — MCU CP");

    /* Init CP core */
    eh_cp_init();

    /* Register per-msg-id callbacks — msg_ids must match host's peer_data_example.c */
    ESP_ERROR_CHECK(eh_cp_feat_peer_data_register_callback(
            MSG_ID_CAT,   on_cat,   &s_cat_ctx));
    ESP_ERROR_CHECK(eh_cp_feat_peer_data_register_callback(
            MSG_ID_DOG,   on_dog,   &s_dog_ctx));
    ESP_ERROR_CHECK(eh_cp_feat_peer_data_register_callback(
            MSG_ID_HUMAN, on_human, &s_human_ctx));

    ESP_LOGI(TAG, "Peer data transfer extension ready — waiting for host messages");

    /* TEMP: keep startup quiet until baseline handshake is verified */
    // const char *hello = "CP ready! (MCU)";
    // eh_cp_feat_peer_data_send(MSG_ID_CP_REPLY,
    //         (const uint8_t *)hello, strlen(hello));
}
