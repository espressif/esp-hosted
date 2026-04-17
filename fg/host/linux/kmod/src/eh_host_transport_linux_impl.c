/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ESP-Hosted Linux Transport Operations Implementation
 *
 * Copyright (C) 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#include "esp_log.h"
#include "esp.h"
#include "esp_if.h"
#include "eh_host_transport_linux_ops.h"

/* Global transport operations pointer */
eh_host_transport_linux_ops_t *g_linux_transport_ops = NULL;

/* Forward declarations from existing code */
extern int esp_init_interface_layer(struct esp_adapter *adapter);
extern int esp_deinit_interface_layer(void);
extern int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb);

/* Global RX ready callback - hooks into existing esp_process_new_packet_intr */
static void (*g_rx_ready_callback)(struct esp_adapter *adapter) = NULL;

/**
 * @brief Linux transport init - wrapper around existing esp_init_interface_layer
 */
static int linux_transport_init(struct esp_adapter *adapter)
{
    if (!adapter)
        return -EINVAL;

    return esp_init_interface_layer(adapter);
}

/**
 * @brief Linux transport deinit - wrapper around existing esp_deinit_interface_layer
 */
static int linux_transport_deinit(struct esp_adapter *adapter)
{
    return esp_deinit_interface_layer();
}

/**
 * @brief Linux transport send SKB - direct call to existing esp_send_packet
 */
static int linux_transport_send_skb(struct esp_adapter *adapter, struct sk_buff *skb)
{
    if (!adapter || !skb)
        return -EINVAL;

    return esp_send_packet(adapter, skb);
}

/**
 * @brief Linux transport receive SKB - direct call to existing if_ops->read
 */
static struct sk_buff* linux_transport_recv_skb(struct esp_adapter *adapter)
{
    if (!adapter || !adapter->if_ops || !adapter->if_ops->read)
        return NULL;

    return adapter->if_ops->read(adapter);
}

/**
 * @brief Register RX ready callback - hooks into esp_process_new_packet_intr
 */
static int linux_transport_register_rx_ready_callback(void (*rx_ready_cb)(struct esp_adapter *adapter))
{
    g_rx_ready_callback = rx_ready_cb;
    return 0;
}

/* Linux transport operations structure */
static eh_host_transport_linux_ops_t linux_transport_ops = {
    .init = linux_transport_init,
    .deinit = linux_transport_deinit,
    .send_skb = linux_transport_send_skb,
    .recv_skb = linux_transport_recv_skb,
    .register_rx_ready_callback = linux_transport_register_rx_ready_callback,
};

/**
 * @brief Initialize Linux transport operations and transport layer
 * Single initialization call that handles everything
 */
int eh_host_transport_linux_ops_init(struct esp_adapter *adapter)
{
    if (!adapter) {
        esp_err("Invalid adapter provided\n");
        return -EINVAL;
    }

    // Register the ops structure
    g_linux_transport_ops = &linux_transport_ops;

    // Set global adapter for transport ops
    g_esp_adapter = adapter;

    // Initialize the actual transport layer
    int ret = esp_init_interface_layer(adapter);
    if (ret != 0) {
        esp_err("Failed to initialize transport layer: %d\n", ret);
        // Cleanup on failure
        g_linux_transport_ops = NULL;
        g_esp_adapter = NULL;
        return ret;
    }

    esp_info("Linux transport ops and layer initialized\n");
    return 0;
}

/**
 * @brief Deinitialize Linux transport operations
 */
void eh_host_transport_linux_ops_deinit(void)
{
    g_linux_transport_ops = NULL;
    g_rx_ready_callback = NULL;
}

/**
 * @brief Enhanced esp_process_new_packet_intr with callback support
 * This function replaces the existing esp_process_new_packet_intr in main.c
 */
void esp_process_new_packet_intr_enhanced(struct esp_adapter *adapter)
{
    /* Call registered callback if available */
    if (g_rx_ready_callback) {
        g_rx_ready_callback(adapter);
    }

    /* Call existing workqueue mechanism */
    if (adapter)
        queue_work(adapter->if_rx_workqueue, &adapter->if_rx_work);
}