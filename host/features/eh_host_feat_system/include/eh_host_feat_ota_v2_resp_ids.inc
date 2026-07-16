#if EH_HOST_FEAT_OTA_READY
case RPC_ID__Resp_OTABegin:
case RPC_ID__Resp_OTAWrite:
case RPC_ID__Resp_OTAEnd:
case RPC_ID__Resp_OTAActivate:
    return rpc_ext_v2_parse_resp_ota(rpc, c);
#endif
