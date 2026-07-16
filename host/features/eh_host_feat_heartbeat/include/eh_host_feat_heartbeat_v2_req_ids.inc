#if EH_HOST_FEAT_HEARTBEAT_READY
case RPC_ID__Req_ConfigHeartbeat:
    return rpc_ext_v2_pick_req_heartbeat(msg_id);
#endif
