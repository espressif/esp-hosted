/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ESP-Hosted Linux Transport Operations
 * Platform-native SKB-based transport interface
 *
 * Copyright (C) 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#ifndef _EH_TRANSPORT_LINUX_OPS_H_
#define _EH_TRANSPORT_LINUX_OPS_H_

#include <linux/skbuff.h>
#include "esp_if.h"
#include "eh_caps.h"

/**
 * @brief Transport power management states
 */
typedef enum {
    ESP_TRANSPORT_POWER_ACTIVE = 0,      /**< Transport fully active */
    ESP_TRANSPORT_POWER_AWAKE,           /**< Transport awake and ready */
    ESP_TRANSPORT_POWER_SLEEPING,        /**< Transport in power save mode */
    ESP_TRANSPORT_POWER_WAKING,          /**< Transport waking up from power save */
    ESP_TRANSPORT_POWER_ERROR            /**< Transport power state error */
} esp_transport_power_state_t;

/**
 * @brief Power management callbacks for main.c to handle power events
 */
typedef struct {
    /**
     * @brief Called when transport enters power save mode
     * @param adapter ESP adapter instance
     * @param user_data User data provided during callback registration
     */
    void (*power_save_start)(struct esp_adapter *adapter, void *user_data);

    /**
     * @brief Called when transport exits power save mode
     * @param adapter ESP adapter instance
     * @param user_data User data provided during callback registration
     */
    void (*power_save_stop)(struct esp_adapter *adapter, void *user_data);
} esp_transport_power_callbacks_t;

/**
 * @brief Linux Transport Power Management Operations (Nested)
 */
typedef struct {
    /**
     * @brief Initialize power management
     * @param adapter ESP adapter instance
     * @return 0 on success, negative error code on failure
     */
    int (*init)(struct esp_adapter *adapter);

    /**
     * @brief Deinitialize power management
     * @param adapter ESP adapter instance
     * @return 0 on success, negative error code on failure
     */
    int (*deinit)(struct esp_adapter *adapter);

    /**
     * @brief Start power save mode
     * @param adapter ESP adapter instance
     * @return 0 on success, negative error code on failure
     */
    int (*start)(struct esp_adapter *adapter);

    /**
     * @brief Stop power save mode (wake up)
     * @param adapter ESP adapter instance
     * @return 0 on success, negative error code on failure
     */
    int (*stop)(struct esp_adapter *adapter);

    /**
     * @brief Get current power state
     * @param adapter ESP adapter instance
     * @return Current power state
     */
    esp_transport_power_state_t (*get_state)(struct esp_adapter *adapter);

    /**
     * @brief Register power event callbacks
     * @param adapter ESP adapter instance
     * @param callbacks Power event callbacks
     * @param user_data User data passed to callbacks
     * @return 0 on success, negative error code on failure
     */
    int (*register_callbacks)(struct esp_adapter *adapter,
                            const esp_transport_power_callbacks_t *callbacks,
                            void *user_data);
} eh_host_linux_power_ops_t;

/**
 * @brief Linux Transport Operations Structure
 *
 * Platform-native operations that work directly with Linux sk_buff structures.
 * These ops preserve the natural Linux kernel networking patterns.
 */
typedef struct {
    /**
     * @brief Initialize transport layer
     * @param adapter ESP adapter instance
     * @return 0 on success, negative error code on failure
     */
    int (*init)(struct esp_adapter *adapter);

    /**
     * @brief Deinitialize transport layer
     * @param adapter ESP adapter instance
     * @return 0 on success, negative error code on failure
     */
    int (*deinit)(struct esp_adapter *adapter);

    /**
     * @brief Send SKB packet (TX path)
     * @param adapter ESP adapter instance
     * @param skb Socket buffer to transmit
     * @return 0 on success, negative error code on failure
     *
     * This directly maps to existing esp_send_packet() functionality
     */
    int (*send_skb)(struct esp_adapter *adapter, struct sk_buff *skb);

    /**
     * @brief Receive SKB packet (RX path)
     * @param adapter ESP adapter instance
     * @return SKB on success, NULL if no packet available
     *
     * This directly maps to existing adapter->if_ops->read() functionality
     */
    struct sk_buff* (*recv_skb)(struct esp_adapter *adapter);

    /**
     * @brief Register RX ready callback
     * @param rx_ready_cb Callback function called when RX data is available
     * @return 0 on success, negative error code on failure
     *
     * This hooks into the existing esp_process_new_packet_intr() mechanism
     */
    int (*register_rx_ready_callback)(void (*rx_ready_cb)(struct esp_adapter *adapter));

    /**
     * @brief Get transport capabilities
     * @param adapter ESP adapter instance
     * @param basic_caps Basic capabilities (ESP_CAPABILITIES)
     * @param ext_caps Extended capabilities (ESP_EXTENDED_CAPABILITIES)
     * @return 0 on success, negative error code on failure
     *
     * Replaces direct process_init_event() calls from main.c
     */
    int (*get_capabilities)(struct esp_adapter *adapter, u32 *basic_caps, u32 *ext_caps);

    /**
     * @brief Get current transport state/status
     * @param adapter ESP adapter instance
     * @param status Transport status flags
     * @return 0 on success, negative error code on failure
     *
     * General purpose state query function
     */
    int (*get_status)(struct esp_adapter *adapter, u32 *status);

    /**
     * @brief Nested power management operations
     *
     * Power management is optional. If NULL, power management is not supported.
     * Follows Linux kernel nested ops pattern (like netdev + ethtool).
     */
    const eh_host_linux_power_ops_t *power_ops;

} eh_host_transport_linux_ops_t;

/**
 * @brief Global Linux transport operations pointer
 *
 * This is set during initialization and used by all transport API calls.
 * Zero overhead dispatch through function pointers.
 */
extern eh_host_transport_linux_ops_t *g_linux_transport_ops;

/**
 * @brief Platform-Native Linux Transport APIs (Zero Overhead Macros)
 *
 * These macros provide unified naming while preserving existing functionality.
 * They expand to direct function calls with no runtime overhead.
 */

/**
 * @brief Initialize Linux transport
 * @param adapter ESP adapter instance
 * @return 0 on success, negative error code on failure
 */
static inline int eh_host_transport_init(struct esp_adapter *adapter)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->init)
        return -EINVAL;
    return g_linux_transport_ops->init(adapter);
}

/**
 * @brief Deinitialize Linux transport
 * @param adapter ESP adapter instance
 * @return 0 on success, negative error code on failure
 */
static inline int eh_host_transport_deinit(struct esp_adapter *adapter)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->deinit)
        return -EINVAL;
    return g_linux_transport_ops->deinit(adapter);
}

/**
 * @brief Send SKB packet through transport
 * @param adapter ESP adapter instance
 * @param skb Socket buffer to send
 * @return 0 on success, negative error code on failure
 *
 * Direct replacement for esp_send_packet(adapter, skb)
 */
static inline int eh_host_transport_send_skb(struct esp_adapter *adapter, struct sk_buff *skb)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->send_skb)
        return -EINVAL;
    return g_linux_transport_ops->send_skb(adapter, skb);
}

/**
 * @brief Receive SKB packet from transport
 * @param adapter ESP adapter instance
 * @return SKB on success, NULL if no packet available
 *
 * Direct replacement for adapter->if_ops->read(adapter)
 */
static inline struct sk_buff* eh_host_transport_recv_skb(struct esp_adapter *adapter)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->recv_skb)
        return NULL;
    return g_linux_transport_ops->recv_skb(adapter);
}

/**
 * @brief Register RX ready notification callback
 * @param rx_ready_cb Callback function for RX ready events
 * @return 0 on success, negative error code on failure
 *
 * Hooks into existing esp_process_new_packet_intr() mechanism
 */
static inline int eh_host_transport_register_rx_ready_callback(void (*rx_ready_cb)(struct esp_adapter *adapter))
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->register_rx_ready_callback)
        return -EINVAL;
    return g_linux_transport_ops->register_rx_ready_callback(rx_ready_cb);
}

/**
 * @brief Get transport capabilities
 * @param adapter ESP adapter instance
 * @param basic_caps Basic capabilities (ESP_CAPABILITIES)
 * @param ext_caps Extended capabilities (ESP_EXTENDED_CAPABILITIES)
 * @return 0 on success, negative error code on failure
 *
 * Replacement for direct process_init_event() usage in main.c
 */
static inline int eh_host_transport_get_capabilities(struct esp_adapter *adapter, u32 *basic_caps, u32 *ext_caps)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->get_capabilities)
        return -EINVAL;
    return g_linux_transport_ops->get_capabilities(adapter, basic_caps, ext_caps);
}

/**
 * @brief Get current transport status
 * @param adapter ESP adapter instance
 * @param status Transport status flags
 * @return 0 on success, negative error code on failure
 */
static inline int eh_host_transport_get_status(struct esp_adapter *adapter, u32 *status)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->get_status)
        return -EINVAL;
    return g_linux_transport_ops->get_status(adapter, status);
}

/**
 * @brief Get current power state
 * @param adapter ESP adapter instance
 * @return Current power state, ESP_TRANSPORT_POWER_ERROR if not supported
 *
 * Replacement for is_host_sleeping() direct calls
 */
static inline esp_transport_power_state_t eh_host_transport_get_power_state(struct esp_adapter *adapter)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->power_ops || !g_linux_transport_ops->power_ops->get_state)
        return ESP_TRANSPORT_POWER_ACTIVE; // Default to active if power management not supported
    return g_linux_transport_ops->power_ops->get_state(adapter);
}

/**
 * @brief Check if transport is sleeping (convenience function)
 * @param adapter ESP adapter instance
 * @return true if sleeping, false otherwise
 *
 * Direct replacement for is_host_sleeping() calls
 */
static inline bool eh_host_transport_is_sleeping(struct esp_adapter *adapter)
{
    return (eh_host_transport_get_power_state(adapter) == ESP_TRANSPORT_POWER_SLEEPING);
}

/**
 * @brief Register power management callbacks
 * @param adapter ESP adapter instance
 * @param callbacks Power event callbacks
 * @param user_data User data passed to callbacks
 * @return 0 on success, negative error code on failure
 */
static inline int eh_host_transport_register_power_callbacks(struct esp_adapter *adapter,
                                                              const esp_transport_power_callbacks_t *callbacks,
                                                              void *user_data)
{
    if (!g_linux_transport_ops || !g_linux_transport_ops->power_ops || !g_linux_transport_ops->power_ops->register_callbacks)
        return -EINVAL;
    return g_linux_transport_ops->power_ops->register_callbacks(adapter, callbacks, user_data);
}

#endif /* _EH_TRANSPORT_LINUX_OPS_H_ */