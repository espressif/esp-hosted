#if EH_HOST_FEAT_FEATURE_CONTROL_READY
case RPC_ID__Resp_FeatureControl:
    return rpc_ext_v2_parse_resp_feature_control(rpc, c);
#endif
