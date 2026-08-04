/* SPDX-License-Identifier: Apache-2.0 */
/* Private (intra-feature) surface for iTWT — not part of the public
 * eh_host_wifi_itwt.h API; shared only between this feature's TUs. */

#ifndef EH_HOST_WIFI_ITWT_PRIV_H_
#define EH_HOST_WIFI_ITWT_PRIV_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Clear all iTWT single-slot event subscribers; called on feature deinit. */
void eh_host_wifi_itwt_reset_subscribers(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_ITWT_PRIV_H_ */
