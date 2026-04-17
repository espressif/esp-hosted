/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_rpc_id_map_v1.h
 *
 * V1 RPC ID range map — single source of truth for V1 request, response,
 * and event ID ranges.  Both CP and host sides include this file.
 *
 * The proto enum values (CTRL_MSG_ID__Req_Base, RPC_ID__Req_Base, etc.) are
 * defined in the generated .pb-c.h files.  Compile-time cross-checks below
 * (guarded by INCLUDE_EH_RPC_ID_MAP_V1_ASSERTS) verify that the ranges here
 * agree with the proto enums.
 */

#pragma once

/* ── Linux FG V1 (CtrlMsg / eh_config.proto) ──────────────────── */
#define EH_RPC_V1_FG_REQ_MIN      100u
#define EH_RPC_V1_FG_REQ_MAX      199u
#define EH_RPC_V1_FG_RESP_MIN     200u
#define EH_RPC_V1_FG_RESP_MAX     299u
#define EH_RPC_V1_FG_EVT_MIN      300u
#define EH_RPC_V1_FG_EVT_MAX      399u

/* ── MCU V1 (Rpc / eh_rpc.proto) ──────────────────────────────── */
#define EH_RPC_V1_MCU_REQ_MIN     0x100u
#define EH_RPC_V1_MCU_REQ_MAX     0x1FFu
#define EH_RPC_V1_MCU_RESP_MIN    0x200u
#define EH_RPC_V1_MCU_RESP_MAX    0x2FFu
#define EH_RPC_V1_MCU_EVT_MIN     0x300u
#define EH_RPC_V1_MCU_EVT_MAX     0x3FFu

/* Compile-time checks against proto-generated enums live in serializer sources
 * to avoid exposing protobuf headers to public users. */
