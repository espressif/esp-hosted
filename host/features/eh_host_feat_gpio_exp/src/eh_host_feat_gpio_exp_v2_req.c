/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_GPIO_EXP_READY

static int compose_req_gpio_config(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                   alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioConfig, req_gpio_config, rpc__req__gpio_config__init);
    RpcGpioConfig *cfg = (RpcGpioConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*cfg));
    if (!cfg) return -1;
    rpc__gpio_config__init(cfg);
    cfg->pin_bit_mask = c->u.gpio_cfg.pin_bit_mask;
    cfg->mode         = c->u.gpio_cfg.mode;
    cfg->pull_up_en   = c->u.gpio_cfg.pull_up_en ? 1 : 0;
    cfg->pull_down_en = c->u.gpio_cfg.pull_down_en ? 1 : 0;
    cfg->intr_type    = c->u.gpio_cfg.intr_type;
    p->config = cfg;
    return 0;
}

static int compose_req_gpio_reset_pin(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioResetPin, req_gpio_reset_pin,
                  rpc__req__gpio_reset_pin__init);
    p->gpio_num = c->u.gpio_num.gpio_num;
    return 0;
}

static int compose_req_gpio_set_level(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioSetLevel, req_gpio_set_level,
                  rpc__req__gpio_set_level__init);
    p->gpio_num = c->u.gpio_level.gpio_num;
    p->level    = c->u.gpio_level.level;
    return 0;
}

static int compose_req_gpio_get_level(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioGetLevel, req_gpio_get_level,
                  rpc__req__gpio_get_level__init);
    p->gpio_num = c->u.gpio_num.gpio_num;
    return 0;
}

static int compose_req_gpio_set_direction(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                          alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioSetDirection, req_gpio_set_direction,
                  rpc__req__gpio_set_direction__init);
    p->gpio_num = c->u.gpio_direction.gpio_num;
    p->mode     = c->u.gpio_direction.mode;
    return 0;
}

static int compose_req_gpio_input_enable(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                         alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioInputEnable, req_gpio_input_enable,
                  rpc__req__gpio_input_enable__init);
    p->gpio_num = c->u.gpio_num.gpio_num;
    return 0;
}

static int compose_req_gpio_set_pull_mode(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                          alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGpioSetPullMode, req_gpio_set_pull_mode,
                  rpc__req__gpio_set_pull_mode__init);
    p->gpio_num = c->u.gpio_pull.gpio_num;
    p->pull     = c->u.gpio_pull.pull;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_gpio_exp(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_GpioConfig:       return compose_req_gpio_config;
    case RPC_ID__Req_GpioResetPin:     return compose_req_gpio_reset_pin;
    case RPC_ID__Req_GpioSetLevel:     return compose_req_gpio_set_level;
    case RPC_ID__Req_GpioGetLevel:     return compose_req_gpio_get_level;
    case RPC_ID__Req_GpioSetDirection: return compose_req_gpio_set_direction;
    case RPC_ID__Req_GpioInputEnable:  return compose_req_gpio_input_enable;
    case RPC_ID__Req_GpioSetPullMode:  return compose_req_gpio_set_pull_mode;
    default:                           return NULL;
    }
}

#endif /* EH_HOST_FEAT_GPIO_EXP_READY */
