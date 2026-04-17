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

#ifndef __APP_PEER_DATA_TRANSFER_H__
#define __APP_PEER_DATA_TRANSFER_H__

/**
 * @file app_peer_data_transfer.h
 * @brief Example-specific structs and declarations for the peer data
 *        transfer demo on the Linux host side.
 *
 * Uses an animal-sound theme to illustrate per-msg-id callbacks with
 * the local_context pattern. The host sends animal sounds to the CP;
 * the CP replies with translated sounds, which are received here.
 */

#include <stdint.h>
#include "ctrl_api.h"
#include "eh_host_ext_peer_data_transfer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Message IDs — must match the CP-side definitions                    */
/* ------------------------------------------------------------------ */

#define PEER_MSG_ID_DOG_SOUND   0x01
#define PEER_MSG_ID_CAT_SOUND   0x02
#define PEER_MSG_ID_BIRD_SOUND  0x03
#define PEER_MSG_ID_CP_REPLY    0x10   /* CP → host reply              */

/* ------------------------------------------------------------------ */
/* Animal context — passed as local_context to each callback           */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *animal_name;
    uint32_t    send_count;
    uint32_t    recv_count;
} animal_ctx_t;

/* ------------------------------------------------------------------ */
/* Demo entry points (called from test.c)                              */
/* ------------------------------------------------------------------ */

/**
 * @brief Demo 1 — Send a dog sound to the CP and wait for its reply.
 * @return SUCCESS / FAILURE
 */
int peer_data_transfer_demo1_dog_sound(void);

/**
 * @brief Demo 2 — Send a cat sound to the CP and wait for its reply.
 * @return SUCCESS / FAILURE
 */
int peer_data_transfer_demo2_cat_sound(void);

/**
 * @brief Demo 3 — Send a bird sound to the CP and wait for its reply.
 * @return SUCCESS / FAILURE
 */
int peer_data_transfer_demo3_bird_sound(void);

/**
 * @brief Incoming event handler — called by the ctrl library when a
 *        CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG arrives from the CP.
 *
 * Dispatches to the registered per-msg-id callback (if any).
 *
 * @param app_event  Event from ctrl library
 * @return SUCCESS / FAILURE
 */
int peer_data_transfer_event_handler(ctrl_cmd_t *app_event);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PEER_DATA_TRANSFER_H__ */
