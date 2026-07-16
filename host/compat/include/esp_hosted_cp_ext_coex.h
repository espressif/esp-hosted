/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for upstream-MCU ext-coex surface. */

#ifndef EH_COMPAT_ESP_HOSTED_CP_EXT_COEX_H_
#define EH_COMPAT_ESP_HOSTED_CP_EXT_COEX_H_

#include "eh_host_cp_ext_coex.h"

#define esp_hosted_ext_coex_work_mode_t       eh_host_cp_ext_coex_work_mode_t
#define esp_hosted_ext_coex_gpio_set_t        eh_host_cp_ext_coex_gpio_set_t

#define ESP_HOSTED_EXT_COEX_LEADER_ROLE       EH_HOST_CP_EXT_COEX_LEADER_ROLE
#define ESP_HOSTED_EXT_COEX_FOLLOWER_ROLE     EH_HOST_CP_EXT_COEX_FOLLOWER_ROLE
#define ESP_HOSTED_EXT_COEX_UNKNOWN_ROLE      EH_HOST_CP_EXT_COEX_UNKNOWN_ROLE

#define ESP_HOSTED_EXT_COEX_WIRE_1            EH_HOST_CP_EXT_COEX_WIRE_1
#define ESP_HOSTED_EXT_COEX_WIRE_2            EH_HOST_CP_EXT_COEX_WIRE_2
#define ESP_HOSTED_EXT_COEX_WIRE_3            EH_HOST_CP_EXT_COEX_WIRE_3
#define ESP_HOSTED_EXT_COEX_WIRE_4            EH_HOST_CP_EXT_COEX_WIRE_4

#define esp_hosted_cp_ext_coex_set_work_mode(m)        eh_host_cp_ext_coex_set_work_mode(m)
#define esp_hosted_cp_ext_coex_set_gpio_pin(w, p)      eh_host_cp_ext_coex_set_gpio_pin((w), (p))
#define esp_hosted_cp_ext_coex_set_grant_delay(u)      eh_host_cp_ext_coex_set_grant_delay(u)
#define esp_hosted_cp_ext_coex_set_validate_high(b)    eh_host_cp_ext_coex_set_validate_high(b)
#define esp_hosted_cp_ext_coex_disable()               eh_host_cp_ext_coex_disable()

#endif /* EH_COMPAT_ESP_HOSTED_CP_EXT_COEX_H_ */
