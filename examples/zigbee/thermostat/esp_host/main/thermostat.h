/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#define ESP_ZIGBEE_PRIMARY_CHANNEL_MASK   ((1U << 13))
#define ESP_ZIGBEE_SECONDARY_CHANNEL_MASK (0x07FFF800U)

#define ESP_ZIGBEE_HA_THERMOSTAT_EP_ID (1)

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"

#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET

#define ESP_ZIGBEE_ZC_CONFIG()                          \
    {                                                   \
        .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR, \
        .install_code_policy = false,                   \
        .zczr_config = {                                \
            .max_children = 10,                         \
        },                                              \
    }

// radio_config done via ESP-Hosted
#define ESP_ZIGBEE_PLATFORM_CONFIG()                                 \
    {                                                                \
        .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
    }

#define ESP_ZIGBEE_DEFAULT_CONFIG()                      \
    {                                                    \
        .device_config = ESP_ZIGBEE_ZC_CONFIG(),         \
        .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
    };
