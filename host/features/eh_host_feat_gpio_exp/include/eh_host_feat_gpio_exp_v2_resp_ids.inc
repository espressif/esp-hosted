#if EH_HOST_FEAT_GPIO_EXP_READY
case RPC_ID__Resp_GpioConfig:
case RPC_ID__Resp_GpioResetPin:
case RPC_ID__Resp_GpioSetLevel:
case RPC_ID__Resp_GpioGetLevel:
case RPC_ID__Resp_GpioSetDirection:
case RPC_ID__Resp_GpioInputEnable:
case RPC_ID__Resp_GpioSetPullMode:
    return rpc_ext_v2_parse_resp_gpio_exp(rpc, c);
#endif
