/* SPDX-License-Identifier: Apache-2.0 */
/*
 * event_groups.h — empty stub for non-FreeRTOS hosts.
 *
 * Upstream IDF's esp_event_legacy.h includes this for FreeRTOS event-group
 * APIs. Linux user-space and other non-FreeRTOS ports skip the legacy event
 * loop entirely (CONFIG_IDF_TARGET_LINUX=1 in esp_hosted_coprocessor_config.h),
 * so this stub just satisfies the compiler when something pulls it in
 * unconditionally.
 *
 * On the esp_idf port real FreeRTOS provides this header.
 */

#ifndef EVENT_GROUPS_H_
#define EVENT_GROUPS_H_

#endif /* EVENT_GROUPS_H_ */
