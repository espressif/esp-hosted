/* SPDX-License-Identifier: GPL-2.0 */

#ifndef NW_HELPER_FUNC_H
#define NW_HELPER_FUNC_H

#include "test.h"

int down_sta_netdev(const network_info_t *info);
int up_sta_netdev(const network_info_t *info);
int down_softap_netdev(const network_info_t *info);
int up_softap_netdev(const network_info_t *info);
int remove_default_gateway(const char *gateway);
int add_default_gateway(const char *gateway);
int remove_dns(const char *dns);
int add_dns(const char *dns);
int set_network_static_ip(int sockfd, const char* iface, const char* ip, const char* netmask, const char* gateway);
int update_host_network_port_range(uint16_t port_start, uint16_t port_end);
int clear_host_network_port_range(void);
int create_socket(int domain, int type, int protocol, int *sock);
int close_socket(int sock);
int interface_up(int sockfd, const char* iface);
int interface_down(int sockfd, const char* iface);
int set_hw_addr(int sockfd, const char* iface, const char* mac);
int convert_mac_to_bytes(uint8_t *out, size_t out_size, const char *s);

#endif