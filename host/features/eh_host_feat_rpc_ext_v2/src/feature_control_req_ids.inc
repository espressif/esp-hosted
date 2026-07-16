#if EH_HOST_FEAT_FEATURE_CONTROL_READY
case RPC_ID__Req_FeatureControl:
    return rpc_ext_v2_pick_req_feature_control(msg_id);
#endif
