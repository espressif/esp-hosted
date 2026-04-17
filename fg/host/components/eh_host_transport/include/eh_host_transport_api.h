/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EH_HOST_TRANSPORT_API_H
#define __EH_HOST_TRANSPORT_API_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Transport agnostic APIs for ESP-Hosted host implementations
 *
 * This header provides common transport APIs that both Linux kernel
 * and MCU implementations must fulfill. The host application code
 * calls these agnostic APIs regardless of underlying platform.
 */

/** Transport Handle Types **/
typedef void* eh_host_transport_handle_t;

/** Transport Events **/
typedef enum {
    EH_TRANSPORT_EVENT_CONNECTED,       ///< Transport connection established
    EH_TRANSPORT_EVENT_DISCONNECTED,    ///< Transport connection lost
    EH_TRANSPORT_EVENT_RX_DATA,         ///< Data received
    EH_TRANSPORT_EVENT_TX_COMPLETE,     ///< Transmission completed
    EH_TRANSPORT_EVENT_ERROR            ///< Transport error occurred
} eh_host_transport_event_t;

/** Transport Types **/
typedef enum {
    EH_TRANSPORT_SPI,                   ///< SPI transport
    EH_TRANSPORT_SDIO,                  ///< SDIO transport
    EH_TRANSPORT_SPI_HD,                ///< SPI Half-Duplex transport
    EH_TRANSPORT_UART,                  ///< UART transport
    EH_TRANSPORT_MAX                    ///< Maximum transport types
} eh_host_transport_type_t;

/** Transport Configuration **/
typedef struct {
    eh_host_transport_type_t type;           ///< Transport type
    void *platform_config;                     ///< Platform-specific configuration
    size_t max_buffer_size;                     ///< Maximum buffer size
    uint32_t timeout_ms;                        ///< Operation timeout
} eh_host_transport_config_t;

/** Transport Callbacks **/
typedef void (*eh_host_transport_event_cb_t)(eh_host_transport_event_t event,
                                                void *event_data,
                                                void *user_data);

typedef void (*eh_host_transport_rx_cb_t)(const uint8_t *data,
                                             size_t len,
                                             void *user_data);

/**
 * @brief Transport Operations Structure
 *
 * Both Linux kernel and MCU implementations must provide these function pointers.
 * This creates a unified interface regardless of underlying platform.
 */
typedef struct {
    /** Initialize transport */
    esp_err_t (*init)(eh_host_transport_handle_t *handle,
                      const eh_host_transport_config_t *config);

    /** Deinitialize transport */
    esp_err_t (*deinit)(eh_host_transport_handle_t handle);

    /** Send data */
    esp_err_t (*send)(eh_host_transport_handle_t handle,
                      const uint8_t *data,
                      size_t len,
                      uint32_t timeout_ms);

    /** Receive data (blocking) */
    esp_err_t (*receive)(eh_host_transport_handle_t handle,
                         uint8_t *buffer,
                         size_t buffer_size,
                         size_t *received_len,
                         uint32_t timeout_ms);

    /** Register event callback */
    esp_err_t (*register_event_callback)(eh_host_transport_handle_t handle,
                                         eh_host_transport_event_cb_t callback,
                                         void *user_data);

    /** Register RX callback (for async operation) */
    esp_err_t (*register_rx_callback)(eh_host_transport_handle_t handle,
                                      eh_host_transport_rx_cb_t callback,
                                      void *user_data);

    /** Get transport status */
    esp_err_t (*get_status)(eh_host_transport_handle_t handle,
                           uint32_t *status);

    /** Reset transport */
    esp_err_t (*reset)(eh_host_transport_handle_t handle);

} eh_host_transport_ops_t;

/**
 * @brief Transport Driver Structure
 *
 * Platform implementations register their operations through this structure
 */
typedef struct {
    const char *name;                           ///< Transport driver name
    eh_host_transport_type_t type;           ///< Transport type supported
    const eh_host_transport_ops_t *ops;      ///< Transport operations
} eh_host_transport_driver_t;

/**
 * ===================================================================
 * INTERNAL STRUCTURES - For Implementation Use Only
 * ===================================================================
 */

/** Forward declaration for inline functions **/
struct esp_transport_instance;
extern struct esp_transport_instance* eh_host_transport_get_instance(void);

/**
 * ===================================================================
 * PUBLIC APIs - Platform Agnostic Transport Interface
 * ===================================================================
 * Host application code uses ONLY these APIs, regardless of platform
 */

/**
 * @brief Initialize transport layer
 *
 * @param config Transport configuration
 * @param handle Output transport handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_init(const eh_host_transport_config_t *config,
                                   eh_host_transport_handle_t *handle);

/**
 * @brief Deinitialize transport layer
 *
 * @param handle Transport handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_deinit(eh_host_transport_handle_t handle);

/**
 * @brief Send data via transport
 *
 * @param handle Transport handle
 * @param data Data buffer to send
 * @param len Data length
 * @param timeout_ms Transmission timeout
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_send(eh_host_transport_handle_t handle,
                                   const uint8_t *data,
                                   size_t len,
                                   uint32_t timeout_ms);

/**
 * @brief Receive data from transport
 *
 * @param handle Transport handle
 * @param buffer Buffer to receive data
 * @param buffer_size Buffer size
 * @param received_len Actual received length
 * @param timeout_ms Reception timeout
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_receive(eh_host_transport_handle_t handle,
                                      uint8_t *buffer,
                                      size_t buffer_size,
                                      size_t *received_len,
                                      uint32_t timeout_ms);

/**
 * @brief Register event callback
 *
 * @param handle Transport handle
 * @param callback Event callback function
 * @param user_data User data for callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_register_event_callback(eh_host_transport_handle_t handle,
                                                      eh_host_transport_event_cb_t callback,
                                                      void *user_data);

/**
 * @brief Register RX callback for async data reception
 *
 * @param handle Transport handle
 * @param callback RX callback function
 * @param user_data User data for callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_register_rx_callback(eh_host_transport_handle_t handle,
                                                   eh_host_transport_rx_cb_t callback,
                                                   void *user_data);

/**
 * @brief Get transport status
 *
 * @param handle Transport handle
 * @param status Output status value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_get_status(eh_host_transport_handle_t handle,
                                         uint32_t *status);

/**
 * @brief Reset transport
 *
 * @param handle Transport handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_reset(eh_host_transport_handle_t handle);

/**
 * ===================================================================
 * PLATFORM REGISTRATION APIs - For Platform Implementation Use Only
 * ===================================================================
 */

/**
 * @brief Register transport driver (for platform implementations)
 *
 * MCU and Linux implementations call this to register their drivers
 *
 * @param driver Transport driver structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_register_driver(const eh_host_transport_driver_t *driver);

/**
 * @brief Unregister transport driver (for platform implementations)
 *
 * @param type Transport type to unregister
 * @return esp_err_t ESP_OK on success
 */
esp_err_t eh_host_transport_unregister_driver(eh_host_transport_type_t type);

/**
 * ===================================================================
 * PERFORMANCE CRITICAL INLINE FUNCTIONS
 * ===================================================================
 * Zero-overhead inline functions for high-frequency operations
 */

/**
 * @brief Send data via transport (PERFORMANCE CRITICAL - INLINED)
 *
 * @param handle Transport handle
 * @param data Data buffer to send
 * @param len Data length
 * @param timeout_ms Transmission timeout
 * @return esp_err_t ESP_OK on success
 */
static inline esp_err_t eh_host_transport_send(eh_host_transport_handle_t handle,
                                                 const uint8_t *data,
                                                 size_t len,
                                                 uint32_t timeout_ms)
{
    struct esp_transport_instance *transport = eh_host_transport_get_instance();

    if (!transport->ops || !transport->ops->send) {
        return ESP_ERR_INVALID_STATE;
    }

    return transport->ops->send(handle, data, len, timeout_ms);
}

/**
 * @brief Receive data from transport (PERFORMANCE CRITICAL - INLINED)
 *
 * @param handle Transport handle
 * @param buffer Buffer to receive data
 * @param buffer_size Buffer size
 * @param received_len Actual received length
 * @param timeout_ms Reception timeout
 * @return esp_err_t ESP_OK on success
 */
static inline esp_err_t eh_host_transport_receive(eh_host_transport_handle_t handle,
                                                    uint8_t *buffer,
                                                    size_t buffer_size,
                                                    size_t *received_len,
                                                    uint32_t timeout_ms)
{
    struct esp_transport_instance *transport = eh_host_transport_get_instance();

    if (!transport->ops || !transport->ops->receive) {
        return ESP_ERR_INVALID_STATE;
    }

    return transport->ops->receive(handle, buffer, buffer_size, received_len, timeout_ms);
}

/**
 * @brief Register event callback (INLINED)
 */
static inline esp_err_t eh_host_transport_register_event_callback(eh_host_transport_handle_t handle,
                                                                    eh_host_transport_event_cb_t callback,
                                                                    void *user_data)
{
    struct esp_transport_instance *transport = eh_host_transport_get_instance();

    if (!transport->ops || !transport->ops->register_event_callback) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return transport->ops->register_event_callback(handle, callback, user_data);
}

/**
 * @brief Register RX callback for async data reception (INLINED)
 */
static inline esp_err_t eh_host_transport_register_rx_callback(eh_host_transport_handle_t handle,
                                                                 eh_host_transport_rx_cb_t callback,
                                                                 void *user_data)
{
    struct esp_transport_instance *transport = eh_host_transport_get_instance();

    if (!transport->ops || !transport->ops->register_rx_callback) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return transport->ops->register_rx_callback(handle, callback, user_data);
}

/**
 * @brief Get transport status (INLINED)
 */
static inline esp_err_t eh_host_transport_get_status(eh_host_transport_handle_t handle,
                                                       uint32_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    struct esp_transport_instance *transport = eh_host_transport_get_instance();

    if (!transport->ops || !transport->ops->get_status) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return transport->ops->get_status(handle, status);
}

/**
 * @brief Reset transport (INLINED)
 */
static inline esp_err_t eh_host_transport_reset(eh_host_transport_handle_t handle)
{
    struct esp_transport_instance *transport = eh_host_transport_get_instance();

    if (!transport->ops || !transport->ops->reset) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return transport->ops->reset(handle);
}

#ifdef __cplusplus
}
#endif

#endif /* __EH_HOST_TRANSPORT_API_H */