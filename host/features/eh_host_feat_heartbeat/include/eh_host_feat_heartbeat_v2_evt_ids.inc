#if EH_HOST_FEAT_HEARTBEAT_READY
case RPC_ID__Event_Heartbeat:
    return rpc_ext_v2_parse_event_heartbeat(rpc, c);
#endif
