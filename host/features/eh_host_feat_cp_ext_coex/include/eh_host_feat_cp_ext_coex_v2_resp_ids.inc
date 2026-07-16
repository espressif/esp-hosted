#if EH_HOST_FEAT_CP_EXT_COEX_READY
case RPC_ID__Resp_ExtCoex:
    return rpc_ext_v2_parse_resp_ext_coex(rpc, c);
#endif
