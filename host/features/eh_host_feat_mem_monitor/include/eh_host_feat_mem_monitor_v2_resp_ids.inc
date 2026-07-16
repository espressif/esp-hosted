#if EH_HOST_FEAT_MEM_MONITOR_READY
case RPC_ID__Resp_MemMonitor:
    return rpc_ext_v2_parse_resp_mem_monitor(rpc, c);
#endif
