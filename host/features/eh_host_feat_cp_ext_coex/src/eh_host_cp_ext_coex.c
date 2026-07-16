/* SPDX-License-Identifier: Apache-2.0 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "eh_host_port.h"
#include "esp_event.h"

#include "eh_host_event.h"
#include "eh_host_cp_ext_coex.h"

#if EH_HOST_FEAT_CP_EXT_COEX_READY

esp_err_t eh_host_cp_ext_coex_set_work_mode(eh_host_cp_ext_coex_work_mode_t work_mode)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.ext_coex.cmd           = RPC__EXT_COEX_CMD__SetWorkMode;
    req->u.ext_coex.set_work_mode = (uint32_t)work_mode;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_ExtCoex, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_cp_ext_coex_set_gpio_pin(uint32_t wire_type,
        const eh_host_cp_ext_coex_gpio_set_t *gpio_pins)
{
    if (!gpio_pins || wire_type > EH_HOST_CP_EXT_COEX_WIRE_4) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.ext_coex.cmd                   = RPC__EXT_COEX_CMD__SetGpioPin;
    req->u.ext_coex.set_gpio_wire_type    = wire_type;
    req->u.ext_coex.set_gpio_request_pin  = gpio_pins->request;
    req->u.ext_coex.set_gpio_priority_pin = gpio_pins->priority;
    req->u.ext_coex.set_gpio_grant_pin    = gpio_pins->grant;
    req->u.ext_coex.set_gpio_tx_line_pin  = gpio_pins->tx_line;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_ExtCoex, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_cp_ext_coex_set_grant_delay(uint8_t delay_us)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.ext_coex.cmd                = RPC__EXT_COEX_CMD__SetGrantDelay;
    req->u.ext_coex.set_grant_delay_us = (uint32_t)delay_us;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_ExtCoex, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_cp_ext_coex_set_validate_high(bool is_high_valid)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.ext_coex.cmd               = RPC__EXT_COEX_CMD__SetValidateHigh;
    req->u.ext_coex.set_validate_high = is_high_valid;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_ExtCoex, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_cp_ext_coex_disable(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.ext_coex.cmd = RPC__EXT_COEX_CMD__Disable;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_ExtCoex, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#endif /* EH_HOST_FEAT_CP_EXT_COEX_READY */
