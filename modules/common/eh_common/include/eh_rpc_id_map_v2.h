/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_rpc_id_map_v2.h
 *
 * V2 RPC ID range map — unified namespace, no FG/MCU split.
 * Both Linux FG and MCU hosts use the same IDs when operating in V2 mode.
 *
 * Range layout (all values ≤ 0x7FFF → 3-byte proto3 varint):
 *   Requests:   0x2000 – 0x3FFF   (8191 IDs)
 *   Responses:  0x4000 – 0x5FFF   (8191 IDs)
 *   Events:     0x6000 – 0x7FFF   (8191 IDs)
 *
 * SENTINEL RULE: The *_MAX values are hard end-of-band sentinels.
 * Do NOT move them when adding new V2 RPC IDs.  Add new IDs before them.
 */

#pragma once

/* ── V2 Unified ──────────────────────────────────────────────────────────── */
#define EH_RPC_V2_REQ_MIN      0x2000u
#define EH_RPC_V2_REQ_MAX      0x3FFFu   /* sentinel — do not move */
#define EH_RPC_V2_RESP_MIN     0x4000u
#define EH_RPC_V2_RESP_MAX     0x5FFFu   /* sentinel — do not move */
#define EH_RPC_V2_EVT_MIN      0x6000u
#define EH_RPC_V2_EVT_MAX      0x7FFFu   /* sentinel — do not move */

/*
 * _Static_assert cross-checks added here once eh_proto_v2
 * schema is finalised (PENDING-006 / Phase F).
 */
