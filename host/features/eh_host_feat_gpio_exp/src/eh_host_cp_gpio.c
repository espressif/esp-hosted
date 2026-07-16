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
#include "eh_host_cp_gpio.h"

#if EH_HOST_FEAT_GPIO_EXP_READY

esp_err_t eh_host_cp_gpio_config(const eh_host_cp_gpio_config_t *cfg)
{
    if (!cfg) {
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }

    req->u.gpio_cfg.pin_bit_mask = cfg->pin_bit_mask;
    req->u.gpio_cfg.mode         = (RpcGpioMode)cfg->mode;
    req->u.gpio_cfg.pull_up_en   = cfg->pull_up_en;
    req->u.gpio_cfg.pull_down_en = cfg->pull_down_en;
    req->u.gpio_cfg.intr_type    = (int32_t)cfg->intr_type;
    req->u.gpio_cfg.drive_cap    = 0;   /* not on wire yet */

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioConfig, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_cp_gpio_reset_pin(uint32_t gpio_num)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }
    req->u.gpio_num.gpio_num = (int32_t)gpio_num;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioResetPin, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_cp_gpio_set_level(uint32_t gpio_num, uint32_t level)
{
    if (level > 1u) {
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }
    req->u.gpio_level.gpio_num = (int32_t)gpio_num;
    req->u.gpio_level.level    = level;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioSetLevel, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_cp_gpio_get_level(uint32_t gpio_num, int *level)
{
    if (!level) {
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }
    req->u.gpio_num.gpio_num = (int32_t)gpio_num;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioGetLevel, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    if (r->resp_event_status != 0) {
        eh_rpc_ctrl_cmd_free(r);
        return ESP_FAIL;
    }
    *level = (int)r->u.gpio_level.level;
    eh_rpc_ctrl_cmd_free(r);
    return ESP_OK;
}

esp_err_t eh_host_cp_gpio_set_direction(uint32_t gpio_num, uint32_t mode)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }
    req->u.gpio_direction.gpio_num = (int32_t)gpio_num;
    req->u.gpio_direction.mode     = (RpcGpioMode)mode;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioSetDirection, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_cp_gpio_input_enable(uint32_t gpio_num)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }
    req->u.gpio_num.gpio_num = (int32_t)gpio_num;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioInputEnable, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_cp_gpio_set_pull_mode(uint32_t gpio_num, uint32_t pull_mode)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) {
        return ESP_FAIL;
    }
    req->u.gpio_pull.gpio_num = (int32_t)gpio_num;
    req->u.gpio_pull.pull     = (RpcGpioPullMode)pull_mode;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GpioSetPullMode, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

#endif /* EH_HOST_FEAT_GPIO_EXP_READY */
