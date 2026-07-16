#if EH_HOST_FEAT_OTA_READY
case RPC_ID__Req_OTABegin:
case RPC_ID__Req_OTAWrite:
case RPC_ID__Req_OTAEnd:
case RPC_ID__Req_OTAActivate:
    return rpc_ext_v2_pick_req_ota(msg_id);
#endif
