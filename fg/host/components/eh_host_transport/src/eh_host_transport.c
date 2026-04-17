/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief ESP-Hosted Transport Host Core Dispatcher Implementation
 *
 * Ultra-simplified: g_transport IS the handle, NULL = not initialized
 * Performance: Static inline functions for critical paths, ~16 bytes total
 */

/** Includes **/
#include "port_eh_host_config.h"
#include "eh_host_transport_api.h"
#include "eh_host_transport.h"

/** Ultra-minimal transport instance (16 bytes) **/
struct esp_transport_instance {
    const eh_host_transport_ops_t *ops;        ///< NULL = not initialized
    void *platform_data;                          ///< Platform-specific data
};

static struct esp_transport_instance g_transport = {0};

/**
 * ===================================================================
 * NON-CRITICAL PATH FUNCTIONS (Regular functions)
 * ===================================================================
 */

esp_err_t eh_host_transport_init(const eh_host_transport_config_t *config,
                                   eh_host_transport_handle_t *handle)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_transport.ops) {
        return ESP_ERR_INVALID_STATE;  // Already initialized
    }

    // Platform-specific initialization based on config->type
#ifdef H_ESP_HOSTED_HOST_PLATFORM_MCU
    esp_err_t ret = eh_host_transport_mcu_init(config);
#elif defined(__linux__)
    esp_err_t ret = eh_host_transport_linux_init(config);
#else
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;
#endif

    if (ret == ESP_OK && handle) {
        *handle = (eh_host_transport_handle_t)&g_transport;  // g_transport IS the handle
    }

    return ret;
}

esp_err_t eh_host_transport_deinit(eh_host_transport_handle_t handle)
{
    if (!g_transport.ops) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;
    if (g_transport.ops->deinit) {
        ret = g_transport.ops->deinit(handle);
    }

    // Clear state - NULL = not initialized
    g_transport.ops = NULL;
    g_transport.platform_data = NULL;

    return ret;
}

/**
 * ===================================================================
 * PLATFORM INTERNAL APIs - For Platform Adapters Only
 * ===================================================================
 */

esp_err_t eh_host_transport_set_ops(const eh_host_transport_ops_t *ops,
                                      void *platform_data)
{
    if (!ops) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_transport.ops) {
        return ESP_ERR_INVALID_STATE;  // Already initialized
    }

    g_transport.ops = ops;
    g_transport.platform_data = platform_data;

    return ESP_OK;
}

void* eh_host_transport_get_platform_data(void)
{
    return g_transport.platform_data;
}

eh_host_transport_handle_t eh_host_transport_get_handle(void)
{
    return (eh_host_transport_handle_t)&g_transport;
}

bool eh_host_transport_is_initialized(void)
{
    return (g_transport.ops != NULL);
}

/**
 * ===================================================================
 * INTERNAL ACCESSOR FOR INLINE FUNCTIONS
 * ===================================================================
 */

struct esp_transport_instance* eh_host_transport_get_instance(void)
{
    return &g_transport;
}