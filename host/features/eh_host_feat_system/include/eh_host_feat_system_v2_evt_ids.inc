#if EH_HOST_FEAT_SYSTEM_READY
case RPC_ID__Event_ESPInit:
    return rpc_ext_v2_parse_event_system(rpc, c);
#endif
