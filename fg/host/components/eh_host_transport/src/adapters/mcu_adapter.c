/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief MCU (ESP-IDF) Transport Adapter
 *
 * Minimal adapter to bridge existing MCU transport_drv.c to unified API.
 * Zero-overhead wrapper around existing transport functions.
 */

#ifdef H_ESP_HOSTED_HOST_PLATFORM_MCU

/** Includes **/
#include "port_eh_host_config.h"
#include "eh_host_transport_api.h"
#include "eh_host_transport.h"

// Include existing MCU transport headers
#include "transport_drv.h"  // Existing MCU transport functions

/** MCU Transport Operations **/
static esp_err_t mcu_transport_init(eh_host_transport_handle_t *handle,
                                   const eh_host_transport_config_t *config);
static esp_err_t mcu_transport_deinit(eh_host_transport_handle_t handle);
static esp_err_t mcu_transport_send(eh_host_transport_handle_t handle,
                                   const uint8_t *data, size_t len, uint32_t timeout_ms);
static esp_err_t mcu_transport_receive(eh_host_transport_handle_t handle,
                                      uint8_t *buffer, size_t buffer_size,
                                      size_t *received_len, uint32_t timeout_ms);
static esp_err_t mcu_transport_register_event_callback(eh_host_transport_handle_t handle,
                                                      eh_host_transport_event_cb_t callback,
                                                      void *user_data);
static esp_err_t mcu_transport_register_rx_callback(eh_host_transport_handle_t handle,
                                                   eh_host_transport_rx_cb_t callback,
                                                   void *user_data);
static esp_err_t mcu_transport_get_status(eh_host_transport_handle_t handle,
                                         uint32_t *status);
static esp_err_t mcu_transport_reset(eh_host_transport_handle_t handle);

/** MCU Transport Operations Table **/
static const eh_host_transport_ops_t mcu_transport_ops = {
    .init = mcu_transport_init,
    .deinit = mcu_transport_deinit,
    .send = mcu_transport_send,
    .receive = mcu_transport_receive,
    .register_event_callback = mcu_transport_register_event_callback,
    .register_rx_callback = mcu_transport_register_rx_callback,
    .get_status = mcu_transport_get_status,
    .reset = mcu_transport_reset,
};

/** Platform Data **/
struct mcu_transport_data {
    eh_host_transport_type_t type;    ///< Transport type
    void *bus_handle;                    ///< Existing bus handle from transport_drv.c
};

static struct mcu_transport_data g_mcu_data = {0};

/**
 * ===================================================================
 * PUBLIC MCU ADAPTER API
 * ===================================================================
 */

/**
 * @brief Initialize MCU transport adapter
 *
 * Called from MCU application instead of direct transport_drv calls
 *
 * @param config Transport configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_mcu_init(const eh_host_transport_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Store configuration
    g_mcu_data.type = config->type;

    // Set unified transport ops
    esp_err_t ret = eh_host_transport_set_ops(&mcu_transport_ops, &g_mcu_data);
    if (ret != ESP_OK) {
        return ret;
    }

    // Call existing MCU transport initialization
    // This would typically call transport_drv_init() or similar
    // For now, just mark as successful - existing code handles transport init

    return ESP_OK;
}

/**
 * @brief Deinitialize MCU transport adapter
 */
esp_err_t eh_host_transport_mcu_deinit(void)
{
    if (eh_host_transport_is_initialized()) {
        // Call existing MCU transport cleanup if needed
        eh_host_transport_set_ops(NULL, NULL);
        g_mcu_data.type = EH_TRANSPORT_MAX;
        g_mcu_data.bus_handle = NULL;
    }
    return ESP_OK;
}

/**
 * ===================================================================
 * TRANSPORT OPERATIONS IMPLEMENTATION
 * ===================================================================
 */

static esp_err_t mcu_transport_init(eh_host_transport_handle_t *handle,
                                   const eh_host_transport_config_t *config)
{
    // Already initialized by eh_host_transport_mcu_init()
    if (handle) {
        *handle = eh_host_transport_get_handle();
    }
    return ESP_OK;
}

static esp_err_t mcu_transport_deinit(eh_host_transport_handle_t handle)
{
    // Cleanup handled by eh_host_transport_mcu_deinit()
    return ESP_OK;
}

static esp_err_t mcu_transport_send(eh_host_transport_handle_t handle,
                                   const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Call existing MCU transport function
    // Note: eh_host_tx expects specific interface parameters
    // For unified API, we use default values suitable for general data
    int ret = eh_host_tx(
        ESP_SERIAL_IF,           // Interface type - serial for general data
        0,                       // Interface number
        (uint8_t *)data,         // Payload buffer
        (uint16_t)len,           // Payload length
        0,                       // Not zero-copy (we copy the data)
        NULL,                    // No buffer to free
        NULL,                    // No free function
        0                        // No special flags
    );

    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t mcu_transport_receive(eh_host_transport_handle_t handle,
                                      uint8_t *buffer, size_t buffer_size,
                                      size_t *received_len, uint32_t timeout_ms)
{
    if (!buffer || !received_len) {
        return ESP_ERR_INVALID_ARG;
    }

    // MCU transport typically uses callback-based RX rather than polling
    // For now, indicate no data available - applications should use RX callbacks
    *received_len = 0;
    return ESP_ERR_TIMEOUT;
}

static esp_err_t mcu_transport_register_event_callback(eh_host_transport_handle_t handle,
                                                      eh_host_transport_event_cb_t callback,
                                                      void *user_data)
{
    // TODO: Bridge to existing MCU event system
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t mcu_transport_register_rx_callback(eh_host_transport_handle_t handle,
                                                   eh_host_transport_rx_cb_t callback,
                                                   void *user_data)
{
    // TODO: Bridge to existing MCU RX callback system
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t mcu_transport_get_status(eh_host_transport_handle_t handle,
                                         uint32_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    // Basic status: initialized = connected
    *status = eh_host_transport_is_initialized() ? 1 : 0;
    return ESP_OK;
}

static esp_err_t mcu_transport_reset(eh_host_transport_handle_t handle)
{
    // TODO: Bridge to existing MCU transport reset mechanism
    return ESP_ERR_NOT_SUPPORTED;
}

#endif /* H_ESP_HOSTED_HOST_PLATFORM_MCU */