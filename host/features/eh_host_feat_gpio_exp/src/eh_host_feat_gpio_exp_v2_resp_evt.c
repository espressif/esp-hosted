/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_GPIO_EXP_READY

int rpc_ext_v2_parse_resp_gpio_exp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_GpioConfig:
        if (!rpc->resp_gpio_config) return -1;
        c->resp_event_status = rpc->resp_gpio_config->resp;
        return 0;
    case RPC_ID__Resp_GpioResetPin:
        if (!rpc->resp_gpio_reset) return -1;
        c->resp_event_status = rpc->resp_gpio_reset->resp;
        return 0;
    case RPC_ID__Resp_GpioSetLevel:
        if (!rpc->resp_gpio_set_level) return -1;
        c->resp_event_status = rpc->resp_gpio_set_level->resp;
        return 0;
    case RPC_ID__Resp_GpioGetLevel:
        if (!rpc->resp_gpio_get_level) return -1;
        c->resp_event_status  = rpc->resp_gpio_get_level->resp;
        c->u.gpio_level.level = rpc->resp_gpio_get_level->level;
        return 0;
    case RPC_ID__Resp_GpioSetDirection:
        if (!rpc->resp_gpio_set_direction) return -1;
        c->resp_event_status = rpc->resp_gpio_set_direction->resp;
        return 0;
    case RPC_ID__Resp_GpioInputEnable:
        if (!rpc->resp_gpio_input_enable) return -1;
        c->resp_event_status = rpc->resp_gpio_input_enable->resp;
        return 0;
    case RPC_ID__Resp_GpioSetPullMode:
        if (!rpc->resp_gpio_set_pull_mode) return -1;
        c->resp_event_status = rpc->resp_gpio_set_pull_mode->resp;
        return 0;
    default:
        return 0;
    }
}

#endif /* EH_HOST_FEAT_GPIO_EXP_READY */
