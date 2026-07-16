#if EH_HOST_FEAT_SYSTEM_READY
case RPC_ID__Req_GetMACAddress:
case RPC_ID__Req_SetMacAddress:
case RPC_ID__Req_IfaceMacAddrSetGet:
case RPC_ID__Req_GetCoprocessorFwVersion:
    return rpc_ext_v2_pick_req_system(msg_id);
#endif
