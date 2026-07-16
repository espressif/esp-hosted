#if EH_HOST_FEAT_PEER_DATA_READY
case RPC_ID__Event_CustomRpc:
    return rpc_ext_v2_parse_event_peer_data(rpc, c);
#endif
