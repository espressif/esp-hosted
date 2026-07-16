#if EH_HOST_FEAT_WIFI_READY
case RPC_ID__Event_WifiEventNoArgs:
case RPC_ID__Event_StaScanDone:
case RPC_ID__Event_StaConnected:
case RPC_ID__Event_StaDisconnected:
case RPC_ID__Event_AP_StaConnected:
case RPC_ID__Event_AP_StaDisconnected:
    return rpc_ext_v2_parse_event_wifi(rpc, c);
#endif
