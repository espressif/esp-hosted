#if EH_HOST_FEAT_CP_EXT_COEX_READY
case RPC_ID__Req_ExtCoex:
    return rpc_ext_v2_pick_req_ext_coex(msg_id);
#endif
