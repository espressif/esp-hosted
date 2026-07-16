/* SPDX-License-Identifier: Apache-2.0 */
#ifndef EH_HOST_FEAT_RPC_EXT_V2_DECODE_PRIV_H_
#define EH_HOST_FEAT_RPC_EXT_V2_DECODE_PRIV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rpc_ext_v2_priv.h"
#include "gen_v2.h"

#define EH_RPC_COPY_BIN(dst, dst_cap, src) do { \
    if ((dst) && (src) && (src)->data && (src)->len) { \
        size_t _n = (src)->len < (dst_cap) ? (src)->len : (dst_cap); \
        memcpy((dst), (src)->data, _n); \
    } \
} while (0)

int rpc_ext_v2_parse_resp_peer_data(const Rpc *rpc,
                                        eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_peer_data(const Rpc *rpc,
                                         eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_nw_split(const Rpc *rpc,
                                       eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_nw_split(const Rpc *rpc,
                                        eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_resp_ota(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_resp_gpio_exp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_resp_ext_coex(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_resp_feature_control(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_wifi(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_wifi(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_wifi_ext_itwt(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_wifi_ext_itwt(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_wifi_ext_dpp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_wifi_ext_dpp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_wifi_ext_ent(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_system(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_system(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_heartbeat(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_heartbeat(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

int rpc_ext_v2_parse_resp_mem_monitor(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);
int rpc_ext_v2_parse_event_mem_monitor(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c);

#endif /* EH_HOST_FEAT_RPC_EXT_V2_DECODE_PRIV_H_ */
