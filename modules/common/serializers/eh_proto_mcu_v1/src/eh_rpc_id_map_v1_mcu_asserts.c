/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Compile-time checks for MCU RPC ID ranges vs proto enums. */

#include "eh_rpc_id_map_v1.h"
#include "eh_rpc.pb-c.h"

_Static_assert(EH_RPC_V1_MCU_REQ_MIN == (unsigned)RPC_ID__Req_Base,
               "MCU REQ_MIN mismatch with proto RPC_ID__Req_Base");
_Static_assert(EH_RPC_V1_MCU_RESP_MIN == (unsigned)RPC_ID__Resp_Base,
               "MCU RESP_MIN mismatch with proto RPC_ID__Resp_Base");
