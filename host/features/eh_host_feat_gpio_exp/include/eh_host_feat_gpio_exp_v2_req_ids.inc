#if EH_HOST_FEAT_GPIO_EXP_READY
case RPC_ID__Req_GpioConfig:
case RPC_ID__Req_GpioResetPin:
case RPC_ID__Req_GpioSetLevel:
case RPC_ID__Req_GpioGetLevel:
case RPC_ID__Req_GpioSetDirection:
case RPC_ID__Req_GpioInputEnable:
case RPC_ID__Req_GpioSetPullMode:
    return rpc_ext_v2_pick_req_gpio_exp(msg_id);
#endif
