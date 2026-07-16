/* SPDX-License-Identifier: Apache-2.0 */
/* Private to eh_host_core sources (incl. per-port reconfigure TUs). */

#ifndef EH_HOST_CORE_INTERNAL_H_
#define EH_HOST_CORE_INTERNAL_H_

#include "sdkconfig.h"
#include <stdint.h>

#ifndef CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_TIMEOUT_MS
#  define CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_TIMEOUT_MS 5000
#endif
#define EH_HOST_CONNECT_DOER_TIMEOUT_MS \
    ((uint32_t)CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_TIMEOUT_MS)

/* Cross-port floor: without it, Linux's 0 default makes every waiter -ETIMEDOUT. */
#define EH_HOST_CONNECT_WAITER_FLOOR_MS 5000u
#define EH_HOST_CONNECT_WAITER_TIMEOUT_MS \
    ((EH_HOST_CONNECT_DOER_TIMEOUT_MS > EH_HOST_CONNECT_WAITER_FLOOR_MS) \
        ? EH_HOST_CONNECT_DOER_TIMEOUT_MS \
        : EH_HOST_CONNECT_WAITER_FLOOR_MS)

#endif /* EH_HOST_CORE_INTERNAL_H_ */
