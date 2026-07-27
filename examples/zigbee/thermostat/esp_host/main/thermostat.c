/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "switch_driver.h"
#include "alarm_timer.h"

#include "esp_zigbee.h"
#include "ezbee/zha.h"

#include "thermostat.h"

#include "esp_hosted.h"
#include "esp_hosted_openthread.h"
#include "esp_hosted_openthread_app.h"

static const char *TAG = "THERMOSTAT";

static void button_event_handler(switch_driver_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != SWITCH_INV_HANDLE, , TAG, "Invalid switch handle");

    uint16_t attr_field[] = {
        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MIN_MEASURED_VALUE_ID,
        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MAX_MEASURED_VALUE_ID, EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_TOLERANCE_ID};
    ezb_zcl_read_attr_cmd_t read_attr_cmd = {
        .cmd_ctrl =
            {
                .dst_addr.addr_mode = EZB_ADDR_MODE_NONE,
                .src_ep             = ESP_ZIGBEE_HA_THERMOSTAT_EP_ID,
                .cluster_id         = EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            },
        .payload.attr_number = sizeof(attr_field) / sizeof(attr_field[0]),
        .payload.attr_field  = attr_field,
    };
    esp_zigbee_lock_acquire(portMAX_DELAY);
    esp_err_t ret = ezb_zcl_read_attr_cmd_req(&read_attr_cmd);
    esp_zigbee_lock_release();
    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Read Attributes of Temperature Measurement");
    } else {
        ESP_LOGE(TAG, "Failed to send ZCL Read Attribute request with error(0x%04x)", ret);
    }
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;

    ESP_RETURN_ON_FALSE(!is_inited, ESP_OK, TAG, "Deferred driver already initialized");

    switch_driver_config_t config = {
        .gpio_num = CONFIG_GPIO_BOOT_ON_DEVKIT,
        .event_cb = button_event_handler,
    };
    switch_driver_handle_t handle = switch_driver_init(&config);
    is_inited                     = handle == SWITCH_INV_HANDLE ? false : true;

    return is_inited ? ESP_OK : ESP_FAIL;
}

static ezb_err_t zdo_config_ha_temperature_sensor_reporting(void)
{
    ezb_err_t                      ret      = EZB_ERR_FAIL;
    ezb_zcl_config_report_record_t record[] = {
        {
            .direction = EZB_ZCL_REPORTING_SEND,
            .attr_id   = EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
            .client =
                {
                    .attr_type    = EZB_ZCL_ATTR_TYPE_INT16,
                    .min_interval = 1,
                    .max_interval = 10,
                    .reportable_change =
                        {
                            .s16 = 200,
                        },
                },
        },
    };
    ezb_zcl_config_report_cmd_t config_report_cmd = {
        .cmd_ctrl =
            {
                .dst_addr.addr_mode = EZB_ADDR_MODE_NONE,
                .src_ep             = ESP_ZIGBEE_HA_THERMOSTAT_EP_ID,
                .cluster_id         = EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            },
        .payload.record_number = sizeof(record) / sizeof(record[0]),
        .payload.record_field  = record,
    };
    ret = ezb_zcl_config_report_cmd_req(&config_report_cmd);
    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Attempt to configure reporting for HA temperature sensor");
    } else {
        ESP_LOGE(TAG, "Failed to configure reporting for HA temperature sensor with error(0x%04x)", ret);
    }
    return ret;
}

static void zdo_bind_ha_temperature_sensor_device_result(const ezb_zdp_bind_req_result_t *result, void *user_ctx)
{
    ezb_zdo_bind_req_t *bind_req = (ezb_zdo_bind_req_t *)user_ctx;

    assert(result);
    if (result->error == EZB_ERR_NONE) {
        if (result->rsp && result->rsp->status == EZB_ZDP_STATUS_SUCCESS) {
            if (bind_req->dst_nwk_addr == ezb_nwk_get_short_address()) {
                ESP_LOGI(TAG, "Bound HA temperature sensor device (0x%04hx, 0x%02x) to local successfully",
                         bind_req->dst_nwk_addr, bind_req->field.dst_ep);
                zdo_config_ha_temperature_sensor_reporting();
            } else {
                ESP_LOGI(TAG, "Subscribed HA temperature sensor device (0x%04hx, 0x%02x) from local successfully",
                         bind_req->dst_nwk_addr, bind_req->field.src_ep);
            }
        } else if (result->rsp) {
            ESP_LOGE(TAG, "Bound HA temperature sensor device failure with status (0x%02x)", result->rsp->status);
        } else {
            ESP_LOGE(TAG, "Bound HA temperature sensor device failure: no response");
        }
    } else {
        ESP_LOGE(TAG, "Bound HA temperature sensor device failure with error (0x%04x)", result->error);
    }
    if (user_ctx) {
        free(user_ctx);
    }
}

static ezb_err_t zdo_bind_ha_temperature_sensor_device(uint16_t dst_short_addr, uint8_t dst_ep)
{
    ezb_err_t           ret          = EZB_ERR_FAIL;
    ezb_zdo_bind_req_t *bind_sensor  = malloc(sizeof(ezb_zdo_bind_req_t));
    ESP_RETURN_ON_FALSE(bind_sensor, EZB_ERR_FAIL, TAG, "No memory for bind request");
    bind_sensor->dst_nwk_addr        = ezb_nwk_get_short_address();
    bind_sensor->field.src_ep        = ESP_ZIGBEE_HA_THERMOSTAT_EP_ID;
    bind_sensor->field.cluster_id    = EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT;
    bind_sensor->field.dst_addr_mode = EZB_ADDR_MODE_EXT;
    bind_sensor->field.dst_ep        = dst_ep;
    bind_sensor->cb                  = zdo_bind_ha_temperature_sensor_device_result;
    bind_sensor->user_ctx            = bind_sensor;

    ezb_nwk_get_extended_address(&bind_sensor->field.src_addr);

    ret = ezb_address_extended_by_short(dst_short_addr, &bind_sensor->field.dst_addr.extended_addr);
    if (ret != EZB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to get extended address for destination device(0x%04hx)", dst_short_addr);
        free(bind_sensor);
        return ret;
    }

    ret = ezb_zdo_bind_req(bind_sensor);
    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Attempt to bind temperature sensor device (0x%04hx, 0x%02x) to local", dst_short_addr, dst_ep);
    } else {
        ESP_LOGE(TAG, "Failed to bind temperature sensor device (0x%04hx, 0x%02x) to local with error(0x%04x)", dst_short_addr,
                 dst_ep, ret);
        free(bind_sensor);
    }

    ezb_zdo_bind_req_t *subscribe_sensor  = malloc(sizeof(ezb_zdo_bind_req_t));
    ESP_RETURN_ON_FALSE(subscribe_sensor, EZB_ERR_FAIL, TAG, "No memory for subscribe request");
    subscribe_sensor->dst_nwk_addr        = dst_short_addr;
    subscribe_sensor->field.src_ep        = dst_ep;
    subscribe_sensor->field.cluster_id    = EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT;
    subscribe_sensor->field.dst_addr_mode = EZB_ADDR_MODE_EXT;
    subscribe_sensor->field.dst_ep        = ESP_ZIGBEE_HA_THERMOSTAT_EP_ID;
    subscribe_sensor->cb                  = zdo_bind_ha_temperature_sensor_device_result;
    subscribe_sensor->user_ctx            = subscribe_sensor;

    ezb_nwk_get_extended_address(&subscribe_sensor->field.dst_addr.extended_addr);

    ret = ezb_address_extended_by_short(dst_short_addr, &subscribe_sensor->field.src_addr);
    if (ret != EZB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to get extended address for destination device(0x%04hx)", dst_short_addr);
        free(subscribe_sensor);
        return ret;
    }

    ret = ezb_zdo_bind_req(subscribe_sensor);
    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Attempt to subscribe temperature sensor device (0x%04hx, 0x%02x) from local", dst_short_addr, dst_ep);
    } else {
        ESP_LOGE(TAG, "Failed to subscribe temperature sensor device (0x%04hx, 0x%02x) from local with error(0x%04x)",
                 dst_short_addr, dst_ep, ret);
        free(subscribe_sensor);
    }

    return ret;
}

static ezb_err_t esp_zigbee_read_manuf_code_and_model_id(uint16_t dst_short_addr, uint8_t dst_ep)
{
    ezb_err_t               ret           = EZB_ERR_FAIL;
    uint16_t                attr_field[]  = {EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID};
    ezb_zcl_read_attr_cmd_t read_attr_cmd = {
        .cmd_ctrl =
            {
                .dst_addr.addr_mode    = EZB_ADDR_MODE_SHORT,
                .src_ep                = ESP_ZIGBEE_HA_THERMOSTAT_EP_ID,
                .dst_addr.u.short_addr = dst_short_addr,
                .dst_ep                = dst_ep,
                .cluster_id            = EZB_ZCL_CLUSTER_ID_BASIC,
            },
        .payload.attr_number = sizeof(attr_field) / sizeof(attr_field[0]),
        .payload.attr_field  = attr_field,
    };
    ret = ezb_zcl_read_attr_cmd_req(&read_attr_cmd);
    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Attempt to read manuf_code and model_id from device (0x%04hx, 0x%02x)", dst_short_addr, dst_ep);
    } else {
        ESP_LOGE(TAG, "Failed to read manuf_code and model_id from device (0x%04hx, 0x%02x) with error(0x%04x)", dst_short_addr,
                 dst_ep, ret);
    }
    return ret;
}

static void zdo_find_ha_temperature_sensor_device_result(const ezb_zdo_match_desc_req_result_t *result, void *user_ctx)
{
    assert(result);
    if (result->error == EZB_ERR_NONE) {
        if (result->rsp && result->rsp->status == EZB_ZDP_STATUS_SUCCESS && result->rsp->match_length > 0 &&
            result->rsp->match_list) {
            for (size_t i = 0; i < result->rsp->match_length; i++) {
                esp_zigbee_read_manuf_code_and_model_id(result->rsp->nwk_addr_of_interest, result->rsp->match_list[i]);
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to find HA temperature sensor device in the network with error(0x%04x)", result->error);
    }
}

static ezb_err_t zdo_find_ha_temperature_sensor_device(uint16_t short_addr)
{
    ezb_err_t ret             = EZB_ERR_FAIL;
    uint16_t  cluster_list[1] = {EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT};

    ezb_zdo_match_desc_req_t req = {
        .dst_nwk_addr = short_addr,
        .field =
            {
                .nwk_addr_of_interest = short_addr,
                .profile_id           = EZB_AF_HA_PROFILE_ID,
                .num_in_clusters      = 1,
                .num_out_clusters     = 0,
                .cluster_list         = cluster_list,
            },
        .cb       = zdo_find_ha_temperature_sensor_device_result,
        .user_ctx = NULL,
    };
    ret = ezb_zdo_match_desc_req(&req);
    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Attempt to find HA temperature sensor device on address(0x%04hx)", short_addr);
    } else {
        ESP_LOGE(TAG, "Failed to find HA temperature sensor device on address(0x%04hx) with error(0x%04x)", short_addr, ret);
    }
    return ret;
}

static void esp_zigbee_alarm_bdb_commissioning(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(arg);
    esp_zigbee_lock_release();
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ezb_bdb_open_network(180);
                ESP_LOGI(TAG, "Device reboot");
            }
        } else {
            ESP_LOGW(TAG, "The %s failed with status(0x%02x), please retry", ezb_app_signal_to_string(signal_type), status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Failed to form network with status(0x%02x)", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Network steering completed");
        } else {
            ESP_LOGW(TAG, "Failed to steering network with status(0x%02x)", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
    } break;
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *dev_annce_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->short_addr);
        zdo_find_ha_temperature_sensor_device(dev_annce_params->short_addr);
    } break;
    case EZB_ZDO_SIGNAL_LEAVE_INDICATION: {
        const ezb_zdo_signal_leave_indication_params_t *leave_ind_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "Zigbee Node(0x%04hx) is leaving network", leave_ind_params->short_addr);
    } break;
    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        if (duration) {
            ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", ezb_nwk_get_panid(), duration);
        } else {
            ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", ezb_nwk_get_panid());
        }
    } break;
    default:
        ESP_LOGI(TAG, "Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

static float zb_s16_to_temperature(int16_t value) { return 1.0 * value / 100; }

static void zcl_core_cmd_read_attr_rsp_handler(ezb_zcl_cmd_read_attr_rsp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, , TAG, "message is empty");
    ESP_LOGI(TAG, "ZCL Read Attribute Response message for endpoint(%d) cluster(0x%04x) %s with status(0x%02x)",
             message->info.dst_ep, message->info.cluster_id,
             message->info.cluster_role == EZB_ZCL_CLUSTER_SERVER ? "server" : "client", message->info.status);

    switch (message->info.cluster_id) {
    case EZB_ZCL_CLUSTER_ID_BASIC: {
        ezb_zcl_read_attr_rsp_variable_t *var = message->in.variables;
        while (var) {
            if (var->status == EZB_ZCL_STATUS_SUCCESS) {
                switch (var->attr_id) {
                case EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID:
                    ESP_LOGI(TAG, "Manufacturer name: %.*s", *(uint8_t *)var->attr_value, (char *)(var->attr_value + 1));
                    break;
                case EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID:
                    ESP_LOGI(TAG, "Model identifier: %.*s", *(uint8_t *)var->attr_value, (char *)(var->attr_value + 1));
                    break;
                default:
                    ESP_LOGI(TAG, "Basic attribute ID: 0x%04x with type: 0x%02x", var->attr_id, var->attr_type);
                    break;
                }
            } else {
                ESP_LOGW(TAG, "Read Basic attribute ID: 0x%04x with status: 0x%02x", var->attr_id, var->status);
            }
            var = var->next;
        }
        zdo_bind_ha_temperature_sensor_device(message->in.header->src_addr.u.short_addr, message->in.header->src_ep);
    } break;
    case EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT: {
        ezb_zcl_read_attr_rsp_variable_t *var = message->in.variables;
        while (var) {
            if (var->status == EZB_ZCL_STATUS_SUCCESS) {
                switch (var->attr_id) {
                case EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID:
                    ESP_LOGI(TAG, "Measured value: %.2f degrees Celsius", zb_s16_to_temperature(*(int16_t *)var->attr_value));
                    break;
                case EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MIN_MEASURED_VALUE_ID:
                    ESP_LOGI(TAG, "Min measured value: %.2f degrees Celsius",
                             zb_s16_to_temperature(*(int16_t *)var->attr_value));
                    break;
                case EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MAX_MEASURED_VALUE_ID:
                    ESP_LOGI(TAG, "Max measured value: %.2f degrees Celsius",
                             zb_s16_to_temperature(*(int16_t *)var->attr_value));
                    break;
                case EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_TOLERANCE_ID:
                    ESP_LOGI(TAG, "Tolerance: %.2f degrees Celsius", zb_s16_to_temperature(*(int16_t *)var->attr_value));
                    break;
                default:
                    ESP_LOGI(TAG, "Temperature Measurement attribute ID: 0x%04x, Value: %p", var->attr_id, var->attr_value);
                    break;
                }
            } else {
                ESP_LOGW(TAG, "Read Temperature Measurement attribute ID: 0x%04x with status: 0x%02x", var->attr_id,
                         var->status);
            }
            var = var->next;
        }
    } break;
    default:
        ESP_LOGW(TAG, "Unknown cluster ID: 0x%04x", message->info.cluster_id);
        break;
    }

}

static void zcl_core_cmd_report_attr_handler(ezb_zcl_cmd_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, , TAG, "message is empty");
    ESP_LOGI(TAG, "ZCL Report Attribute message for endpoint(%d) cluster(0x%04x) %s with status(0x%02x)", message->info.dst_ep,
             message->info.cluster_id, message->info.cluster_role == EZB_ZCL_CLUSTER_SERVER ? "server" : "client",
             message->info.status);

    switch (message->info.cluster_id) {
    case EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT: {
        ezb_zcl_report_attr_variable_t *var = message->in.variables;
        while (var) {
            switch (var->attr_id) {
            case EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID:
                ESP_LOGI(TAG, "Temperature sensor measured value: %.2f degrees Celsius",
                         zb_s16_to_temperature(*(int16_t *)var->attr_value));
                break;
            default:
                ESP_LOGI(TAG, "Temperature Measurement attribute ID: 0x%04x with type: 0x%02x", var->attr_id, var->attr_type);
                break;
            }
            var = var->next;
        }
        break;
    }
    default:
        ESP_LOGW(TAG, "Unknown cluster ID: 0x%04x", message->info.cluster_id);
        break;
    }
}

static void zcl_core_cmd_report_config_rsp_handler(ezb_zcl_cmd_config_report_rsp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, , TAG, "message is empty");
    ESP_LOGI(TAG, "ZCL Report Config Response message for endpoint(%d) cluster(0x%04x) %s with status(0x%02x)",
             message->info.dst_ep, message->info.cluster_id,
             message->info.cluster_role == EZB_ZCL_CLUSTER_SERVER ? "server" : "client", message->info.status);
    ezb_zcl_config_report_rsp_variable_t *var = message->in.variables;
    while (var) {
        if (var->status != EZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Report config attribute ID(0x%04x) with direction(0x%02x)", var->attr_id, var->direction);
        }
        var = var->next;
    }
}

static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
        zcl_core_cmd_read_attr_rsp_handler(message);
        break;
    case EZB_ZCL_CORE_CONFIG_REPORT_RSP_CB_ID:
        zcl_core_cmd_report_config_rsp_handler(message);
        break;
    case EZB_ZCL_CORE_REPORT_ATTR_CB_ID:
        zcl_core_cmd_report_attr_handler(message);
        break;
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        ezb_zcl_cmd_default_rsp_message_t *default_rsp = (ezb_zcl_cmd_default_rsp_message_t *)message;
        ESP_LOGI(TAG, "Received ZCL Default Response: 0x%02x", default_rsp->in.status_code);
    } break;
    default:
        ESP_LOGW(TAG, "ZCL Core Action: ID(0x%04lx)", callback_id);
        break;
    }
}

esp_err_t esp_zigbee_create_zha_thermostat_device(void)
{
    ezb_af_device_desc_t        dev_desc       = ezb_af_create_device_desc();
    ezb_zha_thermostat_config_t thermostat_cfg = EZB_ZHA_THERMOSTAT_CONFIG();
    ezb_af_ep_desc_t            ep_desc        = ezb_zha_create_thermostat(ESP_ZIGBEE_HA_THERMOSTAT_EP_ID, &thermostat_cfg);
    ezb_zcl_cluster_desc_t      basic_desc     = {0};

    basic_desc = ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_basic_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT)));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(
        ep_desc, ezb_zcl_temperature_measurement_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT)));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));

    return ESP_OK;
}

static esp_err_t app_init_esp_hosted(void)
{
	esp_err_t res = ESP_OK;

	res = esp_hosted_init();
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "esp_hosted_init failed");
		return res;
	}

	res = esp_hosted_connect_to_slave();
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "esp_hosted_connect_to_slave failed");
		return res;
	}
	return res;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
	app_init_esp_hosted();

	esp_hosted_openthread_app_init();

    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();

	// get radio config from Hosted
	esp_hosted_openthread_radio_config_t hosted_radio_config = { 0 };
	if (0 != esp_hosted_openthread_get_radio_config(&hosted_radio_config)) {
		ESP_LOGE(TAG, "Failed to get OpenThread radio config from Hosted");
		return;
	}

#if CONFIG_ZB_RADIO_SPINEL_UART
	if (hosted_radio_config.type != HOSTED_OPENTHREAD_TRANSPORT_UART) {
		ESP_LOGE(TAG, "Invalid Hosted Radio Config: expected UART radio config");
		return;
	}
	config.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_UART_RCP;
	esp_hosted_openthread_uart_config_t *src = &hosted_radio_config.radio_uart_config;
	esp_zigbee_uart_config_t *dst = &config.platform_config.radio_config.radio_uart_config;
	dst->port = src->port;
	dst->uart_config.baud_rate  = src->baud_rate;
	dst->uart_config.data_bits  = src->data_bits;
	dst->uart_config.parity     = src->parity;
	dst->uart_config.stop_bits  = src->stop_bits;
	dst->uart_config.flow_ctrl  = src->flow_ctrl;
	dst->uart_config.rx_flow_ctrl_thresh = src->rx_flow_ctrl_thresh;
	dst->uart_config.source_clk = src->source_clk;
	dst->rx_pin = src->rx_pin;
	dst->tx_pin = src->tx_pin;
#else
#error "Unsupported Hosted Radio Config"
#endif

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());

    ESP_ERROR_CHECK(esp_zigbee_create_zha_thermostat_device());

    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));
    ESP_LOGI(TAG, "Start ESP Zigbee Stack");
    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096 * 2, NULL, 5, NULL);
}
