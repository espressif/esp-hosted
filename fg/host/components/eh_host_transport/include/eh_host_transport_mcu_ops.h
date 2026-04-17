/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file eh_host_transport_mcu_ops.h
 * @brief ESP-Hosted MCU Transport Operations
 *
 * Platform-native buffer-based transport interface for MCU (ESP-IDF) hosts.
 * Preserves natural MCU patterns while providing unified naming.
 */

#ifndef _EH_TRANSPORT_MCU_OPS_H_
#define _EH_TRANSPORT_MCU_OPS_H_

#include "esp_err.h"
#include "transport_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MCU Transport Operations Structure
 *
 * Platform-native operations that work directly with buffer-based interfaces.
 * These ops preserve the natural MCU/ESP-IDF patterns and performance.
 */
typedef struct {
    /**
     * @brief Initialize transport layer
     * @param eh_host_up_cb Callback function called when transport is ready
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t (*init)(void(*eh_host_up_cb)(void));

    /**
     * @brief Deinitialize transport layer
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t (*deinit)(void);

    /**
     * @brief Send buffer packet (TX path)
     * @param iface_type Interface type (ESP_STA_IF, ESP_AP_IF, ESP_SERIAL_IF, etc.)
     * @param iface_num Interface number
     * @param data Buffer data to transmit
     * @param len Buffer length
     * @param buff_flags Buffer management flags (H_BUFF_ZEROCOPY, H_BUFF_NO_ZEROCOPY)
     * @param priv_buffer_handle Private buffer handle for zero-copy operations
     * @param free_func_cb Callback to free buffer after transmission
     * @param seq_num Sequence number for packet ordering
     * @return ESP_OK on success, error code on failure
     *
     * This directly maps to existing eh_host_tx() functionality
     */
    int (*send_buffer)(uint8_t iface_type, uint8_t iface_num, uint8_t *data, uint16_t len,
                      uint8_t buff_flags, void *priv_buffer_handle,
                      void (*free_func_cb)(void *priv_buffer_handle), uint8_t seq_num);

    /**
     * @brief Register RX callback for transport channel
     * @param callback RX callback function
     * @param user_data User data passed to callback
     * @return ESP_OK on success, error code on failure
     *
     * This hooks into the existing transport RX task callbacks
     */
    esp_err_t (*register_rx_callback)(transport_channel_rx_fn_t callback, void *user_data);

    /**
     * @brief Get transport status
     * @param status Pointer to store status
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t (*get_status)(uint32_t *status);

} eh_host_transport_mcu_ops_t;

/**
 * @brief Global MCU transport operations pointer
 *
 * This is set during initialization and used by all transport API calls.
 * Zero overhead dispatch through function pointers.
 */
extern eh_host_transport_mcu_ops_t *g_mcu_transport_ops;

/**
 * @brief Platform-Native MCU Transport APIs (Zero Overhead Inline Functions)
 *
 * These inline functions provide unified naming while preserving existing functionality.
 * They compile to direct function calls with no runtime overhead.
 */

/**
 * @brief Initialize MCU transport
 * @param eh_host_up_cb Callback when transport is ready
 * @return ESP_OK on success, error code on failure
 */
static inline esp_err_t eh_host_transport_init(void(*eh_host_up_cb)(void))
{
    if (!g_mcu_transport_ops || !g_mcu_transport_ops->init)
        return ESP_ERR_INVALID_STATE;
    return g_mcu_transport_ops->init(eh_host_up_cb);
}

/**
 * @brief Deinitialize MCU transport
 * @return ESP_OK on success, error code on failure
 */
static inline esp_err_t eh_host_transport_deinit(void)
{
    if (!g_mcu_transport_ops || !g_mcu_transport_ops->deinit)
        return ESP_ERR_INVALID_STATE;
    return g_mcu_transport_ops->deinit();
}

/**
 * @brief Send buffer through transport
 * @param iface_type Interface type
 * @param iface_num Interface number
 * @param data Buffer data
 * @param len Buffer length
 * @param buff_flags Buffer management flags
 * @param priv_buffer_handle Private buffer handle
 * @param free_func_cb Buffer free callback
 * @param seq_num Sequence number
 * @return ESP_OK on success, error code on failure
 *
 * Direct replacement for eh_host_tx(iface_type, iface_num, data, len, buff_flags, priv_buffer_handle, free_func_cb, seq_num)
 */
static inline int eh_host_transport_send_buffer(uint8_t iface_type, uint8_t iface_num,
                                                  uint8_t *data, uint16_t len, uint8_t buff_flags,
                                                  void *priv_buffer_handle,
                                                  void (*free_func_cb)(void *priv_buffer_handle),
                                                  uint8_t seq_num)
{
    if (!g_mcu_transport_ops || !g_mcu_transport_ops->send_buffer)
        return ESP_ERR_INVALID_STATE;
    return g_mcu_transport_ops->send_buffer(iface_type, iface_num, data, len, buff_flags,
                                           priv_buffer_handle, free_func_cb, seq_num);
}

/**
 * @brief Register RX callback
 * @param callback RX callback function
 * @param user_data User data for callback
 * @return ESP_OK on success, error code on failure
 */
static inline esp_err_t eh_host_transport_register_rx_callback(transport_channel_rx_fn_t callback, void *user_data)
{
    if (!g_mcu_transport_ops || !g_mcu_transport_ops->register_rx_callback)
        return ESP_ERR_INVALID_STATE;
    return g_mcu_transport_ops->register_rx_callback(callback, user_data);
}

/**
 * @brief Get transport status
 * @param status Pointer to store status
 * @return ESP_OK on success, error code on failure
 */
static inline esp_err_t eh_host_transport_get_status(uint32_t *status)
{
    if (!g_mcu_transport_ops || !g_mcu_transport_ops->get_status)
        return ESP_ERR_INVALID_STATE;
    return g_mcu_transport_ops->get_status(status);
}

/**
 * @brief Simplified send APIs for common use cases
 */

/**
 * @brief Send serial data (most common MCU use case)
 * @param data Buffer data
 * @param len Buffer length
 * @return ESP_OK on success, error code on failure
 */
static inline int eh_host_transport_send_serial(uint8_t *data, uint16_t len)
{
    return eh_host_transport_send_buffer(ESP_SERIAL_IF, 0, data, len, H_BUFF_NO_ZEROCOPY, NULL, NULL, 0);
}

/**
 * @brief Send STA WiFi data
 * @param data Buffer data
 * @param len Buffer length
 * @return ESP_OK on success, error code on failure
 */
static inline int eh_host_transport_send_sta(uint8_t *data, uint16_t len)
{
    return eh_host_transport_send_buffer(ESP_STA_IF, 0, data, len, H_BUFF_NO_ZEROCOPY, NULL, NULL, 0);
}

/**
 * @brief Send AP WiFi data
 * @param data Buffer data
 * @param len Buffer length
 * @return ESP_OK on success, error code on failure
 */
static inline int eh_host_transport_send_ap(uint8_t *data, uint16_t len)
{
    return eh_host_transport_send_buffer(ESP_AP_IF, 0, data, len, H_BUFF_NO_ZEROCOPY, NULL, NULL, 0);
}

#ifdef __cplusplus
}
#endif

#endif /* _EH_TRANSPORT_MCU_OPS_H_ */
