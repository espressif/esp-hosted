#if EH_HOST_FEAT_MEM_MONITOR_READY
case RPC_ID__Event_MemMonitor:
    return rpc_ext_v2_parse_event_mem_monitor(rpc, c);
#endif
