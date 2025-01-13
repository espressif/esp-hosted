/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _ESP_HOSTED_CLI_H_
#define _ESP_HOSTED_CLI_H_

#include "sdkconfig.h"

/* host */
#ifdef CONFIG_ESP_HOSTED_HOST
#include "esp_hosted_config.h"
#include "power_save_drv.h"
#endif

/* coprocessor */
#ifdef CONFIG_ESP_HOSTED_COPROCESSOR
  #ifdef CONFIG_ESP_HOSTED_CLI_ENABLED
    #include "host_power_save.h"
    #define H_ESP_HOSTED_CLI_ENABLED 1
  #endif /*CONFIG_ESP_HOSTED_CLI_ENABLED*/

  #ifdef CONFIG_SLAVE_MANAGES_WIFI
    #define H_SLAVE_MANAGES_WIFI 1
  #endif
#endif /*CONFIG_ESP_HOSTED_COPROCESSOR*/


#ifdef H_ESP_HOSTED_CLI_ENABLED
int esp_hosted_cli_start();
#endif

#endif /* _ESP_HOSTED_CLI_H_ */
