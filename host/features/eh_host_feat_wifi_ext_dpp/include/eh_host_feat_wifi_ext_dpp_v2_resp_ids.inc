#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
case RPC_ID__Resp_SuppDppInit:
case RPC_ID__Resp_SuppDppDeinit:
case RPC_ID__Resp_SuppDppBootstrapGen:
case RPC_ID__Resp_SuppDppStartListen:
case RPC_ID__Resp_SuppDppStopListen:
    return rpc_ext_v2_parse_resp_wifi_ext_dpp(rpc, c);
#endif
