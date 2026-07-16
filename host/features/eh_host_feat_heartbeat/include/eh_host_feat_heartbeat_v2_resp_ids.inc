#if EH_HOST_FEAT_HEARTBEAT_READY
case RPC_ID__Resp_ConfigHeartbeat:
    return rpc_ext_v2_parse_resp_heartbeat(rpc, c);
#endif
