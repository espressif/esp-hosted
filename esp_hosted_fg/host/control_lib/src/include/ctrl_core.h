/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef __CTRL_CORE_H
#define __CTRL_CORE_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ctrl_api.h"

#define BIT(n) (1UL << (n))

#define MAX_SSID_LENGTH              32
#define MIN_PWD_LENGTH               8
#define MAX_PWD_LENGTH               64
#define MIN_CHNL_NO                  1
#define MAX_CHNL_NO                  11
#define MIN_CONN_NO                  1
#define MAX_CONN_NO                  10

/*
 * Allows user app to create low level protobuf request
 * returns SUCCESS(0) or FAILURE(-1)
 */
int ctrl_app_send_req(ctrl_cmd_t *app_req);

/* When request is sent without an async callback, this function will be called
 * It will wait for control response or timeout for control response
 * This is only used in synchrounous control path
 *
 * Input:
 * > req - control request from user
 *
 * Returns: control response or NULL in case of timeout
 *
 **/
ctrl_cmd_t * ctrl_wait_and_parse_sync_resp(ctrl_cmd_t *req);


/* Checks if async control response callback is available
 * in argument passed of type control request
 *
 * Input:
 * > req - control request from user
 *
 * Returns:
 * > CALLBACK_AVAILABLE - if a non NULL asynchrounous control response
 *                      callback is available
 * In case of failures -
 * > MSG_ID_OUT_OF_ORDER - if request msg id is unsupported
 * > CALLBACK_NOT_REGISTERED - if aync callback is not available
 **/
int is_async_resp_callback_registered(ctrl_cmd_t req);
#endif /* __CTRL_CORE_H */
