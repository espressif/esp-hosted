/* SPDX-License-Identifier: Apache-2.0 */
/* Private WiFi-feature surface — not part of the public eh_host_wifi API.
 * Applications must not call these; they are cross-feature plumbing between
 * the WiFi feature, network-split, and the wifi_remote glue. */

#ifndef EH_HOST_WIFI_PRIV_H_
#define EH_HOST_WIFI_PRIV_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RX gate checked by the glue. Set on STA/AP start/stop; default false
 * blocks RX until the interface is started. is_ap selects AP. */
bool eh_host_wifi_rx_admitted(bool is_ap);

void eh_host_wifi_admit_rx(bool is_ap, bool admit);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_PRIV_H_ */
