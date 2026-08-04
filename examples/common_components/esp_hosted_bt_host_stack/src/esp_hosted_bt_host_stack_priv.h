/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Internal: dispatcher <-> per-stack glue. Each bind/unbind is compiled only
 * when its stack is enabled (idf::bt Kconfig). */
#ifndef ESP_HOSTED_BT_PRIV_H_
#define ESP_HOSTED_BT_PRIV_H_

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)
esp_err_t eh_bt_bind_nimble(void);       /* register RX; TX overrides are link-time */
void      eh_bt_unbind_nimble(void);
#endif

#if defined(CONFIG_BT_BLUEDROID_ENABLED)
esp_err_t eh_bt_bind_bluedroid(void);    /* attach the HCI driver ops */
void      eh_bt_unbind_bluedroid(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ESP_HOSTED_BT_PRIV_H_ */
