/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Compile-time MCU RPC ID range checks vs proto enums (proto *_Max is exclusive). */

#include "eh_rpc_id_map.h"
#include "gen_v2.h"

_Static_assert(EH_RPC_V2_REQ_MIN  == (unsigned)RPC_ID__Req_Base,
               "MCU REQ_MIN mismatch with proto RPC_ID__Req_Base");
_Static_assert(EH_RPC_V2_RESP_MIN == (unsigned)RPC_ID__Resp_Base,
               "MCU RESP_MIN mismatch with proto RPC_ID__Resp_Base");
_Static_assert(EH_RPC_V2_EVT_MIN  == (unsigned)RPC_ID__Event_Base,
               "MCU EVT_MIN mismatch with proto RPC_ID__Event_Base");

/* Containment: proto highest id = *_Max - 1 must be <= inclusive MAP_MAX. */
_Static_assert((unsigned)RPC_ID__Req_Max   <= EH_RPC_V2_REQ_MAX  + 1u,
               "MCU REQ_MAX exceeded — proto assigns ids past 0x1FF");
_Static_assert((unsigned)RPC_ID__Resp_Base + ((unsigned)RPC_ID__Req_Max -
                                              (unsigned)RPC_ID__Req_Base)
               <= EH_RPC_V2_RESP_MAX + 1u,
               "MCU RESP_MAX exceeded — proto resp range past 0x2FF");
_Static_assert((unsigned)RPC_ID__Event_Max <= EH_RPC_V2_EVT_MAX  + 1u,
               "MCU EVT_MAX exceeded — proto assigns event ids past 0x3FF");
