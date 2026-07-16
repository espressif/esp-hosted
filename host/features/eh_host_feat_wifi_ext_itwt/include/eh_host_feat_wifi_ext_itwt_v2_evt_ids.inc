#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
case RPC_ID__Event_StaItwtSetup:
case RPC_ID__Event_StaItwtTeardown:
case RPC_ID__Event_StaItwtSuspend:
case RPC_ID__Event_StaItwtProbe:
    return rpc_ext_v2_parse_event_wifi_ext_itwt(rpc, c);
#endif
