/* SPDX-License-Identifier: Apache-2.0 */
/* External Coexistence — host API + types over Req_ExtCoex (multiplexed by cmd). */

#ifndef EH_HOST_CP_EXT_COEX_H_
#define EH_HOST_CP_EXT_COEX_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EH_HOST_CP_EXT_COEX_LEADER_ROLE   = 0,
    EH_HOST_CP_EXT_COEX_FOLLOWER_ROLE = 2,
    EH_HOST_CP_EXT_COEX_UNKNOWN_ROLE,
} eh_host_cp_ext_coex_work_mode_t;

typedef struct {
    int32_t request;
    int32_t priority;
    int32_t grant;
    int32_t tx_line;
} eh_host_cp_ext_coex_gpio_set_t;

#define EH_HOST_CP_EXT_COEX_WIRE_1   0u
#define EH_HOST_CP_EXT_COEX_WIRE_2   1u
#define EH_HOST_CP_EXT_COEX_WIRE_3   2u
#define EH_HOST_CP_EXT_COEX_WIRE_4   3u

int eh_host_feat_cp_ext_coex_init(void);
int eh_host_feat_cp_ext_coex_deinit(void);

esp_err_t eh_host_cp_ext_coex_set_work_mode(eh_host_cp_ext_coex_work_mode_t work_mode);
esp_err_t eh_host_cp_ext_coex_set_gpio_pin(uint32_t wire_type,
        const eh_host_cp_ext_coex_gpio_set_t *gpio_pins);
esp_err_t eh_host_cp_ext_coex_set_grant_delay(uint8_t delay_us);
esp_err_t eh_host_cp_ext_coex_set_validate_high(bool is_high_valid);
esp_err_t eh_host_cp_ext_coex_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_CP_EXT_COEX_H_ */
