/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Compile-time checks for FG RPC ID ranges vs proto enums. */

#include "eh_rpc_id_map.h"
#include "gen_v1.h"

_Static_assert(EH_RPC_V1_REQ_MIN  == (unsigned)CTRL_MSG_ID__Req_Base,
               "FG REQ_MIN mismatch with proto CTRL_MSG_ID__Req_Base");
_Static_assert(EH_RPC_V1_RESP_MIN == (unsigned)CTRL_MSG_ID__Resp_Base,
               "FG RESP_MIN mismatch with proto CTRL_MSG_ID__Resp_Base");
