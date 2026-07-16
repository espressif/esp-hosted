/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/* eh_rpc_id_map.h — RPC ID range map (single source for req/resp/evt ranges).
 * Covers wire RPC versions V1 (legacy Linux FG CtrlMsg) and V2 (MCU msg_id). */

#pragma once

/* ── V1: Linux FG (CtrlMsg / rpc_v1.proto) ──────────────────── */
#define EH_RPC_V1_REQ_MIN      100u
#define EH_RPC_V1_REQ_MAX      199u
#define EH_RPC_V1_RESP_MIN     200u
#define EH_RPC_V1_RESP_MAX     299u
#define EH_RPC_V1_EVT_MIN      300u
#define EH_RPC_V1_EVT_MAX      399u

/* ── V2: MCU (Rpc / rpc_v2.proto) ──────────────────────────────── */
#define EH_RPC_V2_REQ_MIN     0x100u
#define EH_RPC_V2_REQ_MAX     0x1FFu
#define EH_RPC_V2_RESP_MIN    0x200u
#define EH_RPC_V2_RESP_MAX    0x2FFu
#define EH_RPC_V2_EVT_MIN     0x300u
#define EH_RPC_V2_EVT_MAX     0x3FFu

/* Compile-time cross-checks vs proto enums live in serializer sources. */
