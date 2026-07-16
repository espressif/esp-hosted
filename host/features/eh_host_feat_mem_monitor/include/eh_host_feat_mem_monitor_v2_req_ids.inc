#if EH_HOST_FEAT_MEM_MONITOR_READY
case RPC_ID__Req_MemMonitor:
    return rpc_ext_v2_pick_req_mem_monitor(msg_id);
#endif
