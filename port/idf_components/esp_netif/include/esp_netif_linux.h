/* SPDX-License-Identifier: Apache-2.0 */
/* Linux/kmod-host extensions to esp_netif.h: netdev binding, WIFI_EVENT, DHCP hooks. */
#pragma once

#ifndef ESP_PLATFORM   /* Linux host only */

#include "esp_err.h"
#include "esp_netif_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Override netdev name (default $ESPNETIF_NETDEV or "wlan0").
 *        Call before esp_netif_up().
 */
esp_err_t   esp_netif_linux_set_netdev_name(esp_netif_t *esp_netif,
                                            const char  *linux_ifname);

/** @brief Current netdev name; valid for lifetime of esp_netif. */
const char *esp_netif_linux_get_netdev_name(esp_netif_t *esp_netif);

/**
 * @brief WIFI_EVENT_STA_{CONNECTED,DISCONNECTED} → esp_netif_{up,down}.
 *        Called automatically by esp_netif_create_default_wifi_sta().
 *        Requires esp_event_loop_create_default() first.
 */
esp_err_t   esp_netif_linux_register_wifi_handlers(esp_netif_t *esp_netif);

/**
 * @brief DHCP client / server backend entry points (Linux user-space).
 *
 * Called directly from esp_netif_dhcpc_start/_stop and
 * esp_netif_dhcps_start/_stop in esp_netif_linux.c.  Each backend TU
 * (esp_netif_dhcp_client_linux.c, esp_netif_dhcp_server_linux.c)
 * defines these; the dispatch happens via direct symbol reference,
 * which is what forces the static-library linker to pull the backend
 * TUs in.  No constructor / no hook table / no `--undefined=` sentinel.
 *
 * Returns:
 *   ESP_OK             — backend spawned (or already running)
 *   ESP_ERR_NOT_FOUND  — no executable backend found at compile-/run-time
 *   ESP_ERR_NO_MEM     — allocation failure
 *   other              — backend-specific
 */
esp_err_t eh_dhcpc_start_linux(const char *linux_ifname);
esp_err_t eh_dhcpc_stop_linux (const char *linux_ifname);
esp_err_t eh_dhcps_start_linux(const char *linux_ifname);
esp_err_t eh_dhcps_stop_linux (const char *linux_ifname);

/* Additional non-upstream API: pull this header from
 * protocol_examples_common.h (under #ifndef ESP_PLATFORM) to propagate. */

/** @brief First netif matching fn(netif, ctx). IDF 5.x esp_netif_find_if equivalent. */
esp_netif_t *esp_netif_find_if(bool (*fn)(esp_netif_t *, void *), void *ctx);

/**
 * @brief Enumerate IPv6 addrs. Exposed unconditionally on Linux
 *        (IDF gates by CONFIG_LWIP_IPV6).
 *
 * @param if_ip6  caller-allocated array, >= MAX_IP6_ADDRS_PER_NETIF (5)
 */
int esp_netif_get_all_ip6(esp_netif_t *esp_netif, esp_ip6_addr_t if_ip6[]);

/* IDF gates by CONFIG_LWIP_IPV6; unconditional here for Linux. */
esp_err_t esp_netif_create_ip6_linklocal(esp_netif_t *esp_netif);

/**
 * @brief Lock-free next(); caller serialises. Used in esp_netif_tcpip_exec
 *        callbacks (already serialised by call site on Linux).
 */
esp_netif_t *esp_netif_next_unsafe(esp_netif_t *esp_netif);

#ifdef __cplusplus
}
#endif

#endif /* !ESP_PLATFORM */
