/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for upstream-MCU power-save type + API names. */

#ifndef EH_COMPAT_ESP_HOSTED_POWER_SAVE_H_
#define EH_COMPAT_ESP_HOSTED_POWER_SAVE_H_

#include "eh_host_power_save.h"

#define esp_hosted_wakeup_reason_t           eh_host_wakeup_reason_t
#define esp_hosted_power_save_type_t         eh_host_power_save_type_t

/* Legacy enum tokens are HOSTED_*, not ESP_HOSTED_*. */
#define HOSTED_WAKEUP_UNDEFINED              EH_HOST_WAKEUP_UNDEFINED
#define HOSTED_WAKEUP_NORMAL_REBOOT          EH_HOST_WAKEUP_NORMAL_REBOOT
#define HOSTED_WAKEUP_DEEP_SLEEP             EH_HOST_WAKEUP_DEEP_SLEEP
#define HOSTED_POWER_SAVE_TYPE_NONE          EH_HOST_POWER_SAVE_TYPE_NONE
#define HOSTED_POWER_SAVE_TYPE_LIGHT_SLEEP   EH_HOST_POWER_SAVE_TYPE_LIGHT_SLEEP
#define HOSTED_POWER_SAVE_TYPE_DEEP_SLEEP    EH_HOST_POWER_SAVE_TYPE_DEEP_SLEEP

#define esp_hosted_power_save_init()                eh_host_power_save_init()
#define esp_hosted_power_save_deinit()              eh_host_power_save_deinit()
#define esp_hosted_power_save_enabled()             eh_host_power_save_enabled()
#define esp_hosted_power_saving()                   eh_host_power_saving()
#define esp_hosted_woke_from_power_save()           eh_host_woke_from_power_save()
#define esp_hosted_power_save_start(t)              eh_host_power_save_start(t)
#define esp_hosted_power_save_timer_start(ms)       eh_host_power_save_timer_start(ms)
#define esp_hosted_power_save_timer_stop()          eh_host_power_save_timer_stop()

#endif /* EH_COMPAT_ESP_HOSTED_POWER_SAVE_H_ */
