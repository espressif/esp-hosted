#if EH_HOST_FEAT_PEER_DATA_READY
case RPC_ID__Resp_CustomRpc:
    return rpc_ext_v2_parse_resp_peer_data(rpc, c);
#endif
