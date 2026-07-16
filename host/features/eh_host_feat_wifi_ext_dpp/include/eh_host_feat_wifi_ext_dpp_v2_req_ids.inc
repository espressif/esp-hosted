#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
case RPC_ID__Req_SuppDppInit:
case RPC_ID__Req_SuppDppDeinit:
case RPC_ID__Req_SuppDppBootstrapGen:
case RPC_ID__Req_SuppDppStartListen:
case RPC_ID__Req_SuppDppStopListen:
    return rpc_ext_v2_pick_req_wifi_ext_dpp(msg_id);
#endif
