/*
 * Network Split LWIP Filter Implementation for ESP-Hosted
 *
 * This file provides the packet filtering functionality for network split,
 * allowing both the host and ESP32 to share the same IP address but handle different port ranges.
 */
#ifndef EH_CP_FEAT_NW_SPLIT_H
#define EH_CP_FEAT_NW_SPLIT_H

#include <stdint.h>
#include <stdbool.h>
#include "eh_cp_master_config.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "eh_cp.h"

#if EH_CP_FEAT_NW_SPLIT_READY && defined(CONFIG_LWIP_ENABLE)
/**
 * @brief Enum defining where packets should be routed in network split mode
 */

 typedef enum {
    NW_STACK_COPROCESSOR,  /*!< Route packet to coprocessor (ESP) LWIP stack */
    NW_STACK_HOST,   /*!< Route packet to host LWIP stack */
    NW_STACK_BOTH,   /*!< Route packet to both coprocessor and host LWIP stacks */
    NW_STACK_INVALID /*!< Invalid routing decision */
} eh_cp_feat_nw_split_nw_stack_e;

typedef struct {
    const char *tcp_src_ports;
    const char *tcp_dst_ports;
    const char *udp_src_ports;
    const char *udp_dst_ports;
} eh_cp_feat_nw_split_host_reserved_ports_t;

/**
 * @brief Configuration structure for network split
 */
typedef struct {
    eh_cp_feat_nw_split_nw_stack_e default_nw_stack;
    eh_cp_feat_nw_split_nw_stack_e default_nw_stack_dhcp;
    eh_cp_feat_nw_split_host_reserved_ports_t host_reserved_ports;
    eh_cp_rx_cb_t wlan_sta_rx_callback_for_coprocessor;
} eh_cp_feat_nw_split_config_t;

#if EH_CP_FEAT_NW_SPLIT_READY
#if defined(CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_DHCP_COPROCESSOR)
  #define DEFAULT_NW_STACK_DHCP NW_STACK_COPROCESSOR
#elif defined(CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_DHCP_HOST)
  #define DEFAULT_NW_STACK_DHCP NW_STACK_HOST
#elif defined(CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_DHCP_BOTH)
  #define DEFAULT_NW_STACK_DHCP NW_STACK_BOTH
#else
  #error "Select one of the NW_STACK to forward"
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_ONLY_PORTS_CONFIGURED

#define EH_CP_FEAT_NW_SPLIT_CONFIG_INIT_DEFAULT() { \
    .default_nw_stack_dhcp = DEFAULT_NW_STACK_DHCP, \
    .host_reserved_ports = { \
        .tcp_src_ports = CONFIG_ESP_HOSTED_HOST_TCP_SRC_PORTS, \
        .tcp_dst_ports = CONFIG_ESP_HOSTED_HOST_TCP_DST_PORTS, \
        .udp_src_ports = CONFIG_ESP_HOSTED_HOST_UDP_SRC_PORTS, \
        .udp_dst_ports = CONFIG_ESP_HOSTED_HOST_UDP_DST_PORTS, \
    }, \
    .wlan_sta_rx_callback_for_coprocessor = eh_cp_feat_nw_split_wlan_sta_rx_callback, \
};
#else
#define EH_CP_FEAT_NW_SPLIT_CONFIG_INIT_DEFAULT() { \
    .default_nw_stack_dhcp = DEFAULT_NW_STACK_DHCP, \
    .wlan_sta_rx_callback_for_coprocessor = eh_cp_feat_nw_split_wlan_sta_rx_callback, \
};
#endif
#endif

/**
 * @brief Initialize the network split functionality
 *
 * @param config Configuration for network split
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_nw_split_init(eh_cp_feat_nw_split_config_t *config);

/**
 * @brief Deinitialize network split functionality
 *
 * Unregisters IP event handlers and network split provider.
 * Should be called when network split is no longer needed.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_nw_split_deinit(void);

/* Reset rx override and internal netif state (used during WiFi reinit). */
void eh_cp_feat_nw_split_reset_rx_override(void);
void eh_cp_feat_nw_split_replay_status_if_needed(void);



/**
 * @brief Configure host static port forwarding rules
 *
 * @param ports_str_tcp_src Comma-separated list of TCP source ports to forward to host
 * @param ports_str_tcp_dst Comma-separated list of TCP destination ports to forward to host
 * @param ports_str_udp_src Comma-separated list of UDP source ports to forward to host
 * @param ports_str_udp_dst Comma-separated list of UDP destination ports to forward to host
 * @return ESP_OK on success, error code otherwise
 */
int eh_cp_feat_nw_split_config_host_static_port_forwarding_rules(
    const char *ports_str_tcp_src,
    const char *ports_str_tcp_dst,
    const char *ports_str_udp_src,
    const char *ports_str_udp_dst);

/**
 * @brief Filter and route a packet based on port number
 *
 * This function examines the packet headers and decides whether to route
 * the packet to the coprocessor, host, or both based on port numbers.
 *
 * @param frame_data Pointer to the packet data
 * @param frame_length Length of the packet
 * @return eh_nw_split_nw_stack_e Routing decision
 */
eh_cp_feat_nw_split_nw_stack_e eh_cp_feat_nw_split_filter_packet(void *frame_data, uint16_t frame_length);

/**
 * @brief WLAN station RX callback - For Network Split
 *
 * @param buffer Pointer to the packet data
 * @param len Length of the packet
 * @param eb Event base
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_nw_split_wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);


/* Event subscription for nw_split events is private to each RPC adapter.
 * rpc_mcu and rpc_linux_fg each call esp_event_handler_register directly
 * in their own init() for NW_SPLIT_NETWORK_UP / DOWN.
 * Do NOT expose handler registration here — that was the coupling being removed. */

#endif /* EH_CP_FEAT_NW_SPLIT_READY */
#endif /* EH_CP_FEAT_NW_SPLIT_H */
