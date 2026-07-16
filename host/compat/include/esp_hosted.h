/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_HOSTED_H__
#define __ESP_HOSTED_H__

#include "eh_host_port_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_hosted_api_types.h"
#include "esp_hosted_host_fw_ver.h"
#include "esp_hosted_misc.h"
#include "esp_hosted_event.h"

#if EH_HOST_FEAT_OPENTHREAD_READY
#  include "esp_hosted_openthread.h"
#endif
#if EH_HOST_FEAT_OTA_READY
#  include "esp_hosted_ota.h"
#endif
#if EH_HOST_FEAT_GPIO_EXP_READY
#  include "esp_hosted_cp_gpio.h"
#endif
#if EH_HOST_FEAT_CP_EXT_COEX_READY
#  include "esp_hosted_cp_ext_coex.h"
#endif

typedef struct esp_hosted_transport_config esp_hosted_config_t;

int esp_hosted_init(void);
int esp_hosted_deinit(void);

int esp_hosted_connect_to_slave(void);
int esp_hosted_get_coprocessor_fwversion(esp_hosted_coprocessor_fwver_t *ver_info);
int esp_hosted_get_cp_info(uint32_t *cp_chip_id, char *cp_target_name, size_t cp_target_name_len);

/* buff_zerocopy=0 only today. Linux user-space: only ESP_SERIAL_IF meaningful. */
int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
                  uint8_t *payload_buf, uint16_t payload_len,
                  uint8_t buff_zerocopy,
                  uint8_t *buffer_to_free,
                  void (*free_buf_func)(void *ptr),
                  uint8_t flags);


#ifdef __cplusplus
}
#endif

#endif /* __ESP_HOSTED_H__ */
