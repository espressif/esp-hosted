#if EH_HOST_FEAT_NW_SPLIT_READY
case RPC_ID__Resp_SetDhcpDnsStatus:
case RPC_ID__Resp_GetDhcpDnsStatus:
    return rpc_ext_v2_parse_resp_nw_split(rpc, c);
#endif
