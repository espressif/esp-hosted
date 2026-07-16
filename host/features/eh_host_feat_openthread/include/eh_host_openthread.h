/* SPDX-License-Identifier: Apache-2.0 */
/* OpenThread RCP control surface — host-side API. */

#ifndef EH_HOST_OPENTHREAD_H_
#define EH_HOST_OPENTHREAD_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EH_HOST_OPENTHREAD_QUERY_CONFIGURED = 0,
    EH_HOST_OPENTHREAD_QUERY_INITED,
    EH_HOST_OPENTHREAD_QUERY_ENABLED,
    EH_HOST_OPENTHREAD_QUERY_READY,
} eh_host_openthread_query_t;

esp_err_t eh_host_openthread_rcp_init(void);
esp_err_t eh_host_openthread_rcp_deinit(void);
esp_err_t eh_host_openthread_rcp_start(void);
esp_err_t eh_host_openthread_rcp_stop(void);
esp_err_t eh_host_openthread_rcp_query(eh_host_openthread_query_t query);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_OPENTHREAD_H_ */
