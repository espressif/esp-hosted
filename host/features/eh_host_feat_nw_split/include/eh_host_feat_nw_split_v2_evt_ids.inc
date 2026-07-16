#if EH_HOST_FEAT_NW_SPLIT_READY
case RPC_ID__Event_DhcpDnsStatus:
    return rpc_ext_v2_parse_event_nw_split(rpc, c);
#endif
