/* SPDX-License-Identifier: Apache-2.0 */
/* Host-side mem-monitor lifecycle. init registers Event_MemMonitor
 * handlers onto EH_HOST_EVENT_MEM_MONITOR. */

#ifndef EH_HOST_FEAT_MEM_MONITOR_H_
#define EH_HOST_FEAT_MEM_MONITOR_H_

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_feat_mem_monitor_init(void);
int eh_host_feat_mem_monitor_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_MEM_MONITOR_H_ */
