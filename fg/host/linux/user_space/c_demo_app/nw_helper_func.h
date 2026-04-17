/* SPDX-License-Identifier: GPL-2.0 */

#ifndef NW_HELPER_FUNC_H
#define NW_HELPER_FUNC_H

#define STA_INTERFACE     "ethsta0"
#define AP_INTERFACE      "ethap0"
#define MAC_ADDR_LENGTH   18

#define SUCCESS                      0
#define FAILURE                      -1

/* Network state structure */
typedef struct {
	char mac_addr[MAC_ADDR_LENGTH];
	char ip_addr[MAC_ADDR_LENGTH];
	char netmask[MAC_ADDR_LENGTH];
	char gateway[MAC_ADDR_LENGTH];
	char dns_addr[MAC_ADDR_LENGTH];
	char default_route[MAC_ADDR_LENGTH];
	uint8_t ip_valid;
	uint8_t dns_valid;
	uint8_t network_up;
} network_info_t;

int down_sta_netdev(const network_info_t *info);
int up_sta_netdev(const network_info_t *info);
int up_sta_netdev__with_static_ip_dns_route(const network_info_t *info);

int down_softap_netdev(const network_info_t *info);
int up_softap_netdev(const network_info_t *info);

int remove_default_gateway(const char *gateway);
int add_default_gateway(const char *gateway);
int remove_dns(const char *dns);
int add_dns(const char *dns);
int set_network_static_ip(int sockfd, const char* iface, const char* ip, const char* netmask, const char* gateway);

int create_socket(int domain, int type, int protocol, int *sock);
int close_socket(int sock);

int interface_up(int sockfd, const char* iface);
int interface_down(int sockfd, const char* iface);

int set_hw_addr(int sockfd, const char* iface, const char* mac);
int convert_mac_to_bytes(uint8_t *out, size_t out_size, const char *s);

int update_host_network_port_range(uint16_t port_start, uint16_t port_end);
int clear_host_network_port_range(void);
#endif
