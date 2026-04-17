/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file eh_host_transport_mcu_impl.c
 * @brief ESP-Hosted MCU Transport Operations Implementation
 */

#include "eh_host_transport_mcu_ops.h"
#include "transport_drv.h"
#include "eh_log.h"

DEFINE_LOG_TAG(mcu_transport_ops);

/* Global transport operations pointer */
eh_host_transport_mcu_ops_t *g_mcu_transport_ops = NULL;

/* Forward declarations from existing transport_drv.c */
extern esp_err_t setup_transport(void(*eh_host_up_cb)(void));
extern int eh_host_tx(uint8_t iface_type, uint8_t iface_num, uint8_t *data, uint16_t len,
                        uint8_t buff_flags, void *priv_buffer_handle,
                        void (*free_func_cb)(void *priv_buffer_handle), uint8_t seq_num);

/* Global RX callback storage */
static transport_channel_rx_fn_t g_rx_callback = NULL;
static void *g_rx_user_data = NULL;

/**
 * @brief MCU transport init - wrapper around existing setup_transport
 */
static esp_err_t mcu_transport_init(void(*eh_host_up_cb)(void))
{
    return setup_transport(eh_host_up_cb);
}

/**
 * @brief MCU transport deinit - cleanup function
 */
static esp_err_t mcu_transport_deinit(void)
{
    /* Reset callback */
    g_rx_callback = NULL;
    g_rx_user_data = NULL;

    /* Note: No explicit teardown_transport() found in existing code */
    ESP_LOGI(TAG, "MCU transport deinitialized");
    return ESP_OK;
}

/**
 * @brief MCU transport send buffer - direct call to existing eh_host_tx
 */
static int mcu_transport_send_buffer(uint8_t iface_type, uint8_t iface_num, uint8_t *data, uint16_t len,
                                    uint8_t buff_flags, void *priv_buffer_handle,
                                    void (*free_func_cb)(void *priv_buffer_handle), uint8_t seq_num)
{
    return eh_host_tx(iface_type, iface_num, data, len, buff_flags, priv_buffer_handle, free_func_cb, seq_num);
}

/**
 * @brief Register RX callback - stores callback for transport tasks to use
 */
static esp_err_t mcu_transport_register_rx_callback(transport_channel_rx_fn_t callback, void *user_data)
{
    g_rx_callback = callback;
    g_rx_user_data = user_data;
    ESP_LOGI(TAG, "RX callback registered");
    return ESP_OK;
}

/**
 * @brief Get transport status
 */
static esp_err_t mcu_transport_get_status(uint32_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;

    /* Return basic status - can be enhanced based on transport state */
    *status = (g_mcu_transport_ops != NULL) ? 1 : 0;
    return ESP_OK;
}

/* MCU transport operations structure */
static eh_host_transport_mcu_ops_t mcu_transport_ops = {
    .init = mcu_transport_init,
    .deinit = mcu_transport_deinit,
    .send_buffer = mcu_transport_send_buffer,
    .register_rx_callback = mcu_transport_register_rx_callback,
    .get_status = mcu_transport_get_status,
};

/**
 * @brief Initialize MCU transport operations
 * Call this during component initialization to register the ops
 */
esp_err_t eh_host_transport_mcu_ops_init(void)
{
    g_mcu_transport_ops = &mcu_transport_ops;
    ESP_LOGI(TAG, "MCU transport ops registered");
    return ESP_OK;
}

/**
 * @brief Deinitialize MCU transport operations
 */
void eh_host_transport_mcu_ops_deinit(void)
{
    g_mcu_transport_ops = NULL;
    g_rx_callback = NULL;
    g_rx_user_data = NULL;
    ESP_LOGI(TAG, "MCU transport ops deregistered");
}

/**
 * @brief Get registered RX callback for transport tasks
 * This function is called by transport RX tasks (spi_process_rx_task, etc.)
 */
transport_channel_rx_fn_t eh_host_transport_get_rx_callback(void)
{
    return g_rx_callback;
}

/**
 * @brief Get RX callback user data
 */
void* eh_host_transport_get_rx_user_data(void)
{
    return g_rx_user_data;
}