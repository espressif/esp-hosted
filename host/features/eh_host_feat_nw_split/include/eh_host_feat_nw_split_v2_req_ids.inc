#if EH_HOST_FEAT_NW_SPLIT_READY
case RPC_ID__Req_SetDhcpDnsStatus:
case RPC_ID__Req_GetDhcpDnsStatus:
    return rpc_ext_v2_pick_req_nw_split(msg_id);
#endif
