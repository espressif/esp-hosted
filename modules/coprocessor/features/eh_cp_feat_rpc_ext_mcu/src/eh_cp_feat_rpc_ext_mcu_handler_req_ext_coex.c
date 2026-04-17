/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY

#if EH_CP_FEAT_EXT_COEX_READY

#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#include "eh_cp_transport_gpio_pin_guard.h"
#include "esp_coexist.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rpc_ext_coex";

esp_err_t req_ext_coex(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespExtCoex, resp_ext_coex,
            RpcReqExtCoex, req_ext_coex,
            rpc__resp__ext_coex__init);

#if defined(CONFIG_EXTERNAL_COEX_ENABLE) || defined(CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE)
    switch (req_payload->cmd) {
    case RPC__EXT_COEX_CMD__SetGpioPin: {
        external_coex_wire_t wire_type = (external_coex_wire_t)req_payload->set_gpio_wire_type;
        esp_external_coex_gpio_set_t gpio_pins = {
            .request = (gpio_num_t)req_payload->set_gpio_request_pin,
            .priority = (gpio_num_t)req_payload->set_gpio_priority_pin,
            .grant = (gpio_num_t)req_payload->set_gpio_grant_pin,
            .tx_line = (gpio_num_t)req_payload->set_gpio_tx_line_pin
        };
        if (req_payload->set_gpio_wire_type > EXTERNAL_COEXIST_WIRE_4) {
            resp_payload->resp = ESP_ERR_INVALID_ARG;
            return ESP_OK;
        }

        gpio_num_t coex_pins[] = {
            gpio_pins.request, gpio_pins.priority,
            gpio_pins.grant, gpio_pins.tx_line
        };
        for (int i = 0; i < 4; i++) {
            gpio_num_t p = coex_pins[i];
            if (p < 0 || p == GPIO_NUM_NC) continue;
            if (!eh_cp_transport_gpio_pin_guard_is_eligible(p)) {
                ESP_LOGE(TAG, "GPIO %d reserved by transport", (int)p);
                resp_payload->resp = ESP_ERR_INVALID_ARG;
                return ESP_OK;
            }
        }

        RPC_RET_FAIL_IF(esp_enable_extern_coex_gpio_pin(wire_type, gpio_pins));
        break;
    }
    case RPC__EXT_COEX_CMD__Disable:
        RPC_RET_FAIL_IF(esp_disable_extern_coex_gpio_pin());
        break;
    case RPC__EXT_COEX_CMD__SetWorkMode: {
        esp_extern_coex_work_mode_t work_mode = (esp_extern_coex_work_mode_t)req_payload->set_work_mode;
        RPC_RET_FAIL_IF(esp_external_coex_set_work_mode(work_mode));
        break;
    }
#if defined(SOC_EXTERNAL_COEX_ADVANCE) && SOC_EXTERNAL_COEX_ADVANCE
    case RPC__EXT_COEX_CMD__SetGrantDelay:
        RPC_RET_FAIL_IF(esp_external_coex_set_grant_delay((uint8_t)req_payload->set_grant_delay_us));
        break;
    case RPC__EXT_COEX_CMD__SetValidateHigh:
        RPC_RET_FAIL_IF(esp_external_coex_set_validate_high(req_payload->set_validate_high));
        break;
#else
    case RPC__EXT_COEX_CMD__SetGrantDelay:
    case RPC__EXT_COEX_CMD__SetValidateHigh:
        resp_payload->resp = ESP_ERR_NOT_SUPPORTED;
        break;
#endif
    default:
        resp_payload->resp = ESP_ERR_INVALID_ARG;
        break;
    }
#else
    resp_payload->resp = ESP_ERR_NOT_SUPPORTED;
#endif
    return ESP_OK;
}

#endif /* EH_CP_FEAT_EXT_COEX_READY */
#endif /* EH_CP_FEAT_RPC_MCU_READY */
