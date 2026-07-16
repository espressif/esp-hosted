#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
case RPC_ID__Event_SuppDppUriReady:
case RPC_ID__Event_SuppDppCfgRecvd:
case RPC_ID__Event_SuppDppFail:
case RPC_ID__Event_WifiDppUriReady:
case RPC_ID__Event_WifiDppCfgRecvd:
case RPC_ID__Event_WifiDppFail:
    return rpc_ext_v2_parse_event_wifi_ext_dpp(rpc, c);
#endif
