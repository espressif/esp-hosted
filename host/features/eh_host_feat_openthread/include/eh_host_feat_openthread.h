/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_HOST_FEAT_OPENTHREAD_H_
#define EH_HOST_FEAT_OPENTHREAD_H_

#include "esp_err.h"
#include "eh_host_openthread.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_feat_openthread_init(void);
esp_err_t eh_host_feat_openthread_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_OPENTHREAD_H_ */
