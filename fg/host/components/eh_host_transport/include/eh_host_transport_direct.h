/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EH_HOST_TRANSPORT_DIRECT_H
#define __EH_HOST_TRANSPORT_DIRECT_H

/**
 * @brief ESP-Hosted Transport Host Direct Mapping
 *
 * Ultra-minimal approach: Direct function mapping to existing implementations
 * Zero overhead, maximum compatibility, minimal code changes
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "eh_host_transport_api.h"
#include "port_eh_host_config.h"

/**
 * ===================================================================
 * LINUX DIRECT MAPPING - Maps to existing Linux kmod functions
 * ===================================================================
 */
#ifdef __linux__

#include <linux/skbuff.h>
#include "esp.h"
#include "esp_if.h"

// Global adapter - exposed by main.c
extern struct esp_adapter *g_esp_adapter;

// Helper functions for sk_buff conversion
static inline struct sk_buff* esp_create_skb_from_data(const uint8_t *data, size_t len)
{
    struct sk_buff *skb = dev_alloc_skb(len);
    if (skb) {
        skb_put_data(skb, data, len);
    }
    return skb;
}

static inline int esp_copy_skb_to_buffer(struct sk_buff *skb, uint8_t *buffer,
                                        size_t buffer_size, size_t *received_len)
{
    if (!skb || !buffer || !received_len) return -EINVAL;

    size_t copy_len = min(buffer_size, (size_t)skb->len);
    memcpy(buffer, skb->data, copy_len);
    *received_len = copy_len;
    dev_kfree_skb(skb);
    return 0;
}

// Direct API mappings - ALL APIs consistently mapped
#define eh_host_transport_linux_init(adapter) \
    (g_esp_adapter = (adapter), esp_init_interface_layer(adapter))

#define eh_host_transport_linux_deinit() \
    (esp_deinit_interface_layer(), g_esp_adapter = NULL)

static inline esp_err_t eh_host_transport_send(eh_host_transport_handle_t handle,
                                                 const uint8_t *data, size_t len,
                                                 uint32_t timeout_ms)
{
    if (!g_esp_adapter || !g_esp_adapter->if_ops || !g_esp_adapter->if_ops->write)
        return ESP_ERR_INVALID_STATE;

    struct sk_buff *skb = esp_create_skb_from_data(data, len);
    if (!skb) return ESP_ERR_NO_MEM;

    int ret = g_esp_adapter->if_ops->write(g_esp_adapter, skb);
    if (ret != 0) {
        dev_kfree_skb(skb);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static inline esp_err_t eh_host_transport_receive(eh_host_transport_handle_t handle,
                                                    uint8_t *buffer, size_t buffer_size,
                                                    size_t *received_len, uint32_t timeout_ms)
{
    if (!g_esp_adapter || !g_esp_adapter->if_ops || !g_esp_adapter->if_ops->read)
        return ESP_ERR_INVALID_STATE;

    struct sk_buff *skb = g_esp_adapter->if_ops->read(g_esp_adapter);
    if (!skb) {
        *received_len = 0;
        return ESP_ERR_TIMEOUT;
    }

    int ret = esp_copy_skb_to_buffer(skb, buffer, buffer_size, received_len);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

// Consistent mapping for other APIs
#define eh_host_transport_get_status(handle, status) \
    (*(status) = (g_esp_adapter && g_esp_adapter->if_ops) ? 1 : 0, ESP_OK)

#define eh_host_transport_reset(handle) ESP_ERR_NOT_SUPPORTED

#define eh_host_transport_register_event_callback(handle, callback, user_data) ESP_ERR_NOT_SUPPORTED
#define eh_host_transport_register_rx_callback(handle, callback, user_data) ESP_ERR_NOT_SUPPORTED

#endif /* __linux__ */

/**
 * ===================================================================
 * MCU DIRECT MAPPING - Maps to existing MCU transport functions
 * ===================================================================
 */
#ifdef H_ESP_HOSTED_HOST_PLATFORM_MCU

#include "transport_drv.h"

// Direct API mappings - ALL APIs consistently mapped
#define eh_host_transport_mcu_init(config) \
    setup_transport(NULL)

#define eh_host_transport_mcu_deinit() \
    teardown_transport()

static inline esp_err_t eh_host_transport_send(eh_host_transport_handle_t handle,
                                                 const uint8_t *data, size_t len,
                                                 uint32_t timeout_ms)
{
    // Direct call to existing MCU function
    int ret = eh_host_tx(ESP_SERIAL_IF, 0, (uint8_t *)data, (uint16_t)len, 0, NULL, NULL, 0);
    return (ret == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t eh_host_transport_receive(eh_host_transport_handle_t handle,
                                                    uint8_t *buffer, size_t buffer_size,
                                                    size_t *received_len, uint32_t timeout_ms)
{
    // MCU uses callback-based RX - polling not supported
    *received_len = 0;
    return ESP_ERR_TIMEOUT;
}

// Consistent mapping for other APIs
#define eh_host_transport_get_status(handle, status) \
    (*(status) = 1, ESP_OK)  // Always connected if initialized

#define eh_host_transport_reset(handle) ESP_ERR_NOT_SUPPORTED

#define eh_host_transport_register_event_callback(handle, callback, user_data) ESP_ERR_NOT_SUPPORTED
#define eh_host_transport_register_rx_callback(handle, callback, user_data) ESP_ERR_NOT_SUPPORTED

#endif /* H_ESP_HOSTED_HOST_PLATFORM_MCU */

/**
 * ===================================================================
 * UNIFIED INIT API - Platform Detection
 * ===================================================================
 */
static inline esp_err_t eh_host_transport_init(const eh_host_transport_config_t *config,
                                                  eh_host_transport_handle_t *handle)
{
    if (!config) return ESP_ERR_INVALID_ARG;

#ifdef __linux__
    int ret = eh_host_transport_linux_init((struct esp_adapter *)config->platform_config);
    if (handle) *handle = (eh_host_transport_handle_t)g_esp_adapter;
    return (ret == 0) ? ESP_OK : ESP_FAIL;
#elif defined(H_ESP_HOSTED_HOST_PLATFORM_MCU)
    esp_err_t ret = eh_host_transport_mcu_init(config);
    if (handle) *handle = (eh_host_transport_handle_t)1; // Dummy handle
    return ret;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static inline esp_err_t eh_host_transport_deinit(eh_host_transport_handle_t handle)
{
#ifdef __linux__
    eh_host_transport_linux_deinit();
    return ESP_OK;
#elif defined(H_ESP_HOSTED_HOST_PLATFORM_MCU)
    return eh_host_transport_mcu_deinit();
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* __EH_HOST_TRANSPORT_DIRECT_H */