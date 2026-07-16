/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_EXT_V2_READY

#if EH_CP_FEAT_GPIO_EXP_READY

#include "eh_cp_feat_rpc_ext_v2_priv.h"
#include "eh_cp_transport_gpio_pin_guard.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rpc_gpio";

esp_err_t req_gpio_config(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioConfig, resp_gpio_config,
            RpcReqGpioConfig, req_gpio_config,
            rpc__resp__gpio_config__init);

    gpio_config_t config = {0};
    config.mode = req_payload->config->mode;
    config.pull_up_en = req_payload->config->pull_up_en;
    config.pull_down_en = req_payload->config->pull_down_en;
    config.intr_type = req_payload->config->intr_type;
    config.pin_bit_mask = req_payload->config->pin_bit_mask;
    ESP_LOGD(TAG, "RPC req_gpio_config: mask=0x%llx mode=%u pu=%u pd=%u intr=%u",
             (unsigned long long)config.pin_bit_mask,
             (unsigned)config.mode,
             (unsigned)config.pull_up_en,
             (unsigned)config.pull_down_en,
             (unsigned)config.intr_type);

    if (config.intr_type != GPIO_INTR_DISABLE) {
		ESP_LOGE(TAG, "ISR is not supported from slave yet. please contact through github issues, if needed.");
        resp_payload->resp = ESP_ERR_NOT_SUPPORTED;
        return ESP_OK;
    }

    for (int pin = 0; pin < GPIO_NUM_MAX; ++pin) {
        if (config.pin_bit_mask & (1ULL << pin)) {
            if (!eh_cp_transport_gpio_pin_guard_is_eligible((gpio_num_t)pin)) {
                ESP_LOGE(TAG, "GPIO %d reserved by transport", pin);
                resp_payload->resp = ESP_ERR_INVALID_ARG;
                return ESP_OK;
            }
        }
    }

    RPC_RET_FAIL_IF(gpio_config(&config));
    return ESP_OK;
}

esp_err_t req_gpio_reset(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioResetPin, resp_gpio_reset,
            RpcReqGpioResetPin, req_gpio_reset_pin,
            rpc__resp__gpio_reset_pin__init);

    gpio_num_t gpio_num = req_payload->gpio_num;
    ESP_LOGD(TAG, "RPC req_gpio_reset: gpio=%d", (int)gpio_num);
    if (!eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)) {
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        return ESP_OK;
    }
    RPC_RET_FAIL_IF(gpio_reset_pin(gpio_num));
    return ESP_OK;
}

esp_err_t req_gpio_set_level(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioSetLevel, resp_gpio_set_level,
            RpcReqGpioSetLevel, req_gpio_set_level,
            rpc__resp__gpio_set_level__init);

    gpio_num_t gpio_num = req_payload->gpio_num;
    uint32_t level = req_payload->level;
    ESP_LOGD(TAG, "RPC req_gpio_set_level: gpio=%d level=%u",
             (int)gpio_num, (unsigned)level);
    if (!eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)) {
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        return ESP_OK;
    }
    RPC_RET_FAIL_IF(gpio_set_level(gpio_num, level));
    return ESP_OK;
}

esp_err_t req_gpio_get_level(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioGetLevel, resp_gpio_get_level,
            RpcReqGpioGetLevel, req_gpio_get_level,
            rpc__resp__gpio_get_level__init);

    gpio_num_t gpio_num = req_payload->gpio_num;
    ESP_LOGD(TAG, "RPC req_gpio_get_level: gpio=%d", (int)gpio_num);
    if (!eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)) {
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        return ESP_OK;
    }
    resp_payload->level = gpio_get_level(gpio_num);
    return ESP_OK;
}

esp_err_t req_gpio_set_direction(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioSetDirection, resp_gpio_set_direction,
            RpcReqGpioSetDirection, req_gpio_set_direction,
            rpc__resp__gpio_set_direction__init);

    gpio_num_t gpio_num = req_payload->gpio_num;
    gpio_mode_t mode = req_payload->mode;
    ESP_LOGD(TAG, "RPC req_gpio_set_direction: gpio=%d mode=%u",
             (int)gpio_num, (unsigned)mode);
    if (!eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)) {
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        return ESP_OK;
    }
    RPC_RET_FAIL_IF(gpio_set_direction(gpio_num, mode));
    return ESP_OK;
}

esp_err_t req_gpio_input_enable(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioInputEnable, resp_gpio_input_enable,
            RpcReqGpioInputEnable, req_gpio_input_enable,
            rpc__resp__gpio_input_enable__init);

    gpio_num_t gpio_num = req_payload->gpio_num;
    ESP_LOGD(TAG, "RPC req_gpio_input_enable: gpio=%d", (int)gpio_num);
    if (!eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)) {
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        return ESP_OK;
    }
    RPC_RET_FAIL_IF(gpio_input_enable(gpio_num));
    return ESP_OK;
}

esp_err_t req_gpio_set_pull_mode(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespGpioSetPullMode, resp_gpio_set_pull_mode,
            RpcReqGpioSetPullMode, req_gpio_set_pull_mode,
            rpc__resp__gpio_set_pull_mode__init);

    gpio_num_t gpio_num = req_payload->gpio_num;
    gpio_pull_mode_t pull_mode = req_payload->pull;
    ESP_LOGD(TAG, "RPC req_gpio_set_pull_mode: gpio=%d pull=%u",
             (int)gpio_num, (unsigned)pull_mode);
    if (!eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num)) {
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        return ESP_OK;
    }
    RPC_RET_FAIL_IF(gpio_set_pull_mode(gpio_num, pull_mode));
    return ESP_OK;
}

#endif /* EH_CP_FEAT_GPIO_EXP_READY */
#endif /* EH_CP_FEAT_RPC_EXT_V2_READY */
