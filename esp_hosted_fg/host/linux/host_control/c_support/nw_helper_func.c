#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/route.h>
#include <net/if.h>
#include <linux/if_arp.h>

#include "nw_helper_func.h"

#define ENABLE_DEBUG_LOGS 0
#define ALLOW_ROUTE_UPDATE 1

#if ENABLE_DEBUG_LOGS
  #define DEBUG_LOG_VERBOSE(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_LOG_VERBOSE(fmt, ...)
#endif

int down_sta_netdev(const network_info_t *info) {
	int ret = SUCCESS, sockfd = 0;

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, STA_INTERFACE);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("%s interface down\n", STA_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

#if ALLOW_ROUTE_UPDATE
	ret = remove_default_gateway(info->default_route);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("Default gateway removed: Gateway=%s\n", info->default_route);
	} else {
		DEBUG_LOG_VERBOSE("Failed to remove default gateway\n");
		goto close_sock;
	}
#endif
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}

int up_sta_netdev(const network_info_t *info)
{
	int ret = SUCCESS, sockfd = 0;
	char mac_copy[MAC_ADDR_LENGTH];
	char ip_copy[MAC_ADDR_LENGTH];
	char nm_copy[MAC_ADDR_LENGTH];
	char gw_copy[MAC_ADDR_LENGTH];

	if (!info || info->mac_addr[0] == '\0') {
		printf("Failure: station mac is empty\n");
		return FAILURE;
	}

	if (info->ip_addr[0] == '\0' || info->netmask[0] == '\0' || info->gateway[0] == '\0' ||
		strcmp(info->ip_addr, "0.0.0.0") == 0 || strcmp(info->netmask, "0.0.0.0") == 0 ||
		strcmp(info->gateway, "0.0.0.0") == 0) {
		printf("Invalid network conf to set\n");
		return FAILURE;
	}

	/* Create local copies to handle const qualifiers */
	strncpy(mac_copy, info->mac_addr, MAC_ADDR_LENGTH);
	strncpy(ip_copy, info->ip_addr, MAC_ADDR_LENGTH);
	strncpy(nm_copy, info->netmask, MAC_ADDR_LENGTH);
	strncpy(gw_copy, info->gateway, MAC_ADDR_LENGTH);

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, STA_INTERFACE);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("%s interface down\n", STA_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = set_hw_addr(sockfd, STA_INTERFACE, mac_copy);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("MAC address %s set to %s interface\n", mac_copy, STA_INTERFACE);
	} else {
		printf("Unable to set MAC address to %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = interface_up(sockfd, STA_INTERFACE);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("%s interface up\n", STA_INTERFACE);
	} else {
		printf("Unable to up %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = set_network_static_ip(sockfd, STA_INTERFACE, ip_copy, nm_copy, gw_copy);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("Static IP set: IP=%s, Netmask=%s, Gateway=%s\n", ip_copy, nm_copy, gw_copy);
	} else {
		printf("Failed to set static IP\n");
		goto close_sock;
	}

#if ALLOW_ROUTE_UPDATE
	ret = add_default_gateway(gw_copy);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("Default gateway added: Gateway=%s\n", gw_copy);
	} else {
		printf("Failed to add default gateway\n");
		goto close_sock;
	}
#endif

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}

int down_softap_netdev(const network_info_t *info)
{
	int ret = SUCCESS, sockfd = 0;

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, AP_INTERFACE);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("%s interface down\n", AP_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

#if ALLOW_ROUTE_UPDATE
	ret = remove_default_gateway(info->default_route);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("Default gateway removed: Gateway=%s", info->default_route);
	} else {
		DEBUG_LOG_VERBOSE("Failed to remove default gateway");
		goto close_sock;
	}
#endif

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}

int up_softap_netdev(const network_info_t *info)
{
	int ret = SUCCESS, sockfd = 0;
	char mac_copy[MAC_ADDR_LENGTH];

	if (!info || info->mac_addr[0] == '\0') {
		printf("Failure: softap mac is empty\n");
		return FAILURE;
	}

	/* Create local copy to handle const qualifier */
	strncpy(mac_copy, info->mac_addr, MAC_ADDR_LENGTH);

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, AP_INTERFACE);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("%s interface down\n", AP_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = set_hw_addr(sockfd, AP_INTERFACE, mac_copy);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("MAC address %s set to %s interface\n", mac_copy, AP_INTERFACE);
	} else {
		printf("Unable to set MAC address to %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = interface_up(sockfd, AP_INTERFACE);
	if (ret == SUCCESS) {
		DEBUG_LOG_VERBOSE("%s interface up\n", AP_INTERFACE);
	} else {
		printf("Unable to up %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}


#define MAX_INTERFACE_LEN			 IFNAMSIZ
#define MAC_SIZE_BYTES				 6
#define MIN_MAC_STR_LEN				 17

 /* Function ups in given interface */
int interface_up(int sockfd, const char* iface)
{
	int ret = SUCCESS;
	struct ifreq req = {0};
	size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

	if (!iface) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		printf("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	req.ifr_flags |= IFF_UP;
	ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
	if (ret < 0) {
		return FAILURE;
	}
	return SUCCESS;
}



/* Function to add default gateway */
int add_default_gateway(const char* gateway)
{
	int ret = SUCCESS;
	struct rtentry route = {0};
	int sockfd = 0;

	if (!gateway) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	/* Create socket */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		perror("socket creation failed:");
		return FAILURE;
	}

	/* Setup route entry */
	struct sockaddr_in *addr = (struct sockaddr_in*) &route.rt_gateway;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = inet_addr(gateway);

	addr = (struct sockaddr_in*) &route.rt_dst;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = INADDR_ANY;

	addr = (struct sockaddr_in*) &route.rt_genmask;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = INADDR_ANY;

	route.rt_flags = RTF_UP | RTF_GATEWAY;
	route.rt_metric = 0;

	/* Add the route */
	ret = ioctl(sockfd, SIOCADDRT, &route);
	if (ret < 0) {
		#if ENABLE_DEBUG_LOGS
			perror("add route failed:");
		#endif
		close(sockfd);
		return FAILURE;
	}

	close(sockfd);
	return SUCCESS;
}

/* Function to remove default gateway */
int remove_default_gateway(const char* gateway)
{
	int ret = SUCCESS;
	struct rtentry route = {0};
	int sockfd = 0;

	if (!gateway) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	/* Create socket */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		perror("socket creation failed:");
		return FAILURE;
	}

	/* Setup route entry */
	struct sockaddr_in *addr = (struct sockaddr_in*) &route.rt_gateway;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = inet_addr(gateway);

	addr = (struct sockaddr_in*) &route.rt_dst;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = INADDR_ANY;

	addr = (struct sockaddr_in*) &route.rt_genmask;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = INADDR_ANY;

	route.rt_flags = RTF_UP | RTF_GATEWAY;
	route.rt_metric = 0;

	/* Remove the route */
	ret = ioctl(sockfd, SIOCDELRT, &route);
	if (ret < 0) {
		#if ENABLE_DEBUG_LOGS
			perror("remove route failed:");
		#endif
		close(sockfd);
		return FAILURE;
	}

	close(sockfd);
	return SUCCESS;
}


int set_network_static_ip(int sockfd, const char* iface, const char* ip, const char* netmask, const char* gateway)
{
	int ret = SUCCESS;
	struct ifreq req = {0};
	struct sockaddr_in* addr = NULL;
	size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

	if (!iface || !ip || !netmask || !gateway) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		memcpy(req.ifr_name, iface, if_name_len);
		req.ifr_name[if_name_len] = '\0';
	} else {
		printf("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	/* Set IP address */
	addr = (struct sockaddr_in*)&req.ifr_addr;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = inet_addr(ip);
	ret = ioctl(sockfd, SIOCSIFADDR, &req);
	if (ret < 0) {
		perror("set ip address:");
		return FAILURE;
	}

	/* Set netmask */
	addr = (struct sockaddr_in*)&req.ifr_netmask;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = inet_addr(netmask);
	ret = ioctl(sockfd, SIOCSIFNETMASK, &req);
	if (ret < 0) {
		perror("set netmask:");
		return FAILURE;
	}

	/* Set gateway */
	addr = (struct sockaddr_in*)&req.ifr_dstaddr;
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = inet_addr(gateway);
	ret = ioctl(sockfd, SIOCSIFDSTADDR, &req);
	if (ret < 0) {
		#if ENABLE_DEBUG_LOGS
			perror("set gateway:");
		#endif
		return FAILURE;
	}

	return SUCCESS;
}

/* Function sets static DNS server */
int add_dns(const char* dns)
{
	int ret = SUCCESS;
	FILE *resolv = NULL;
	char line[256];
	char dns_entry[256];
	int dns_exists = 0;

	if (!dns) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	/* Format DNS entry string */
	snprintf(dns_entry, sizeof(dns_entry), "nameserver %s", dns);

	/* First check if DNS entry already exists */
	resolv = fopen("/etc/resolv.conf", "r");
	if (resolv) {
		while (fgets(line, sizeof(line), resolv)) {
			/* Remove newline */
			line[strcspn(line, "\n")] = 0;
			if (strcmp(line, dns_entry) == 0) {
				dns_exists = 1;
				break;
			}
		}
		fclose(resolv);
	}

	if (!dns_exists) {
		/* Append new DNS entry */
		resolv = fopen("/etc/resolv.conf", "a");
		if (!resolv) {
			perror("open resolv.conf:");
			return FAILURE;
		}

		ret = fprintf(resolv, "%s\n", dns_entry);
		if (ret < 0) {
			perror("write dns server:");
			fclose(resolv);
			return FAILURE;
		}
		fclose(resolv);
	}

	return SUCCESS;
}


/* Function removes DNS entry from resolv.conf */
int remove_dns(const char *dns)
{
	int ret = SUCCESS;
	FILE *resolv = NULL, *tmp = NULL;
	char line[256];
	char dns_entry[256];

	if (!dns) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	/* Format DNS entry string */
	snprintf(dns_entry, sizeof(dns_entry), "nameserver %s", dns);

	/* Open original file for reading and temp file for writing */
	resolv = fopen("/etc/resolv.conf", "r");
	if (!resolv) {
		perror("open resolv.conf:");
		return FAILURE;
	}

	tmp = fopen("/etc/resolv.conf.tmp", "w");
	if (!tmp) {
		perror("open resolv.conf.tmp:");
		fclose(resolv);
		return FAILURE;
	}

	/* Copy all lines except matching DNS entry */
	while (fgets(line, sizeof(line), resolv)) {
		/* Remove newline */
		line[strcspn(line, "\n")] = 0;
		if (strcmp(line, dns_entry) != 0) {
			fprintf(tmp, "%s\n", line);
		}
	}

	fclose(resolv);
	fclose(tmp);

	/* Replace original with temp file */
	ret = rename("/etc/resolv.conf.tmp", "/etc/resolv.conf");
	if (ret < 0) {
		perror("rename resolv.conf:");
		return FAILURE;
	}

	return SUCCESS;
}


 /* Function downs in given interface */
int interface_down(int sockfd, const char* iface)
{
	int ret = SUCCESS;
	struct ifreq req = {0};
	size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

	if (!iface) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		printf("Failed: Max interface len allowed- %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	req.ifr_flags &= ~IFF_UP;
	ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
	if (ret < 0) {
		#if ENABLE_DEBUG_LOGS
			perror("interface down:");
		#endif
		return FAILURE;
	}
	return SUCCESS;
}



 /* Function converts mac string to byte stream */
int convert_mac_to_bytes(uint8_t *out, size_t out_size, const char *s)
{
	int mac[MAC_SIZE_BYTES] = {0};
	int num_bytes = 0;
	if (!s || (strlen(s) < MIN_MAC_STR_LEN) || (out_size < MAC_SIZE_BYTES))  {
		if (!s) {
			printf("empty input mac str\n");
		} else if (strlen(s)<MIN_MAC_STR_LEN) {
			printf("strlen of in str [%zu]<MIN_MAC_STR_LEN[%u]\n",
					strlen(s), MIN_MAC_STR_LEN);
		} else {
			printf("out_size[%zu]<MAC_SIZE_BYTES[%u]\n",
					out_size, MAC_SIZE_BYTES);
		}
		return FAILURE;
	}

	num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
			&mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

	if ((num_bytes < (MAC_SIZE_BYTES - 1))	||
		(mac[0] > 0xFF) ||
		(mac[1] > 0xFF) ||
		(mac[2] > 0xFF) ||
		(mac[3] > 0xFF) ||
		(mac[4] > 0xFF) ||
		(mac[5] > 0xFF)) {
		printf("failed\n");
		return FAILURE;
	}

	out[0] = mac[0]&0xff;
	out[1] = mac[1]&0xff;
	out[2] = mac[2]&0xff;
	out[3] = mac[3]&0xff;
	out[4] = mac[4]&0xff;
	out[5] = mac[5]&0xff;
	return SUCCESS;
}


 /* Function sets mac address to given interface */
int set_hw_addr(int sockfd, const char* iface, const char* mac)
{
	int ret = SUCCESS;
	struct ifreq req = {0};
	char mac_bytes[MAC_SIZE_BYTES] = "";
	size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

	if (!iface || !mac) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		printf("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	memset(mac_bytes, '\0', MAC_SIZE_BYTES);
	ret = convert_mac_to_bytes((uint8_t *)&mac_bytes, sizeof(mac_bytes), mac);

	if (ret) {
		printf("Failed to convert mac address \n");
		return FAILURE;
	}

	req.ifr_hwaddr.sa_family = ARPHRD_ETHER;
	memcpy(req.ifr_hwaddr.sa_data, mac_bytes, MAC_SIZE_BYTES);
	ret = ioctl(sockfd, SIOCSIFHWADDR, &req);

	if (ret < 0) {
		return FAILURE;
	}
	return SUCCESS;
}

 /* Function creates an endpoint for communication and
  * returns a file descriptor (integer number) that
  * refers to that endpoint */
int create_socket(int domain, int type, int protocol, int *sock)
{
	if (!sock) {
		printf("Invalid parameter\n");
		return FAILURE;
	}

	*sock = socket(domain, type, protocol);
	if (*sock < 0)
	{
		printf("Failure to open socket\n");
		return FAILURE;
	}
	return SUCCESS;
}

 /* Function closes an endpoint for communication */
int close_socket(int sock)
{
	int ret;
	ret = close(sock);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;
}

int update_host_network_port_range(uint16_t port_start, uint16_t port_end)
{
	FILE *fp = NULL;
	char line[256];
	int found = 0;
	char temp_file[] = "/tmp/sysctl.conf.XXXXXX";
	int temp_fd;
	FILE *temp_fp = NULL;
	char current_start[16], current_end[16];

	/* Open sysctl.conf file */
	fp = fopen("/etc/sysctl.conf", "r");
	if (!fp) {
		printf("Failed to open /etc/sysctl.conf: %s\n", strerror(errno));
		return FAILURE;
	}

	/* Check if entry exists with same values */
	while (fgets(line, sizeof(line), fp)) {
		if (strstr(line, "net.ipv4.ip_local_port_range")) {
			if (sscanf(line, "net.ipv4.ip_local_port_range = %s %s", current_start, current_end) == 2) {
				if (atoi(current_start) == port_start && atoi(current_end) == port_end) {
					printf("Port range already set to %u-%u\n", port_start, port_end);
					fclose(fp);
					return SUCCESS;
				}
			}
		}
	}
	rewind(fp);

	/* Create temp file */
	temp_fd = mkstemp(temp_file);
	if (temp_fd < 0) {
		printf("Failed to create temp file: %s\n", strerror(errno));
		fclose(fp);
		return FAILURE;
	}

	temp_fp = fdopen(temp_fd, "w");
	if (!temp_fp) {
		printf("Failed to open temp file: %s\n", strerror(errno));
		close(temp_fd);
		fclose(fp);
		return FAILURE;
	}

	/* Update file contents */
	while (fgets(line, sizeof(line), fp)) {
		if (strstr(line, "net.ipv4.ip_local_port_range")) {
			fprintf(temp_fp, "net.ipv4.ip_local_port_range = %u %u\n", port_start, port_end);
			found = 1;
		} else {
			fputs(line, temp_fp);
		}
	}

	/* Add entry if not found */
	if (!found) {
		fprintf(temp_fp, "net.ipv4.ip_local_port_range = %u %u\n", port_start, port_end);
	}

	fclose(fp);
	fclose(temp_fp);

	/* Replace original with temp file */
	if (rename(temp_file, "/etc/sysctl.conf") != 0) {
		printf("Failed to update sysctl.conf: %s\n", strerror(errno));
		unlink(temp_file);
		return FAILURE;
	}

	/* Apply changes */
	if (system("sysctl -p") != 0) {
		printf("Failed to apply sysctl changes\n");
		return FAILURE;
	}

	printf("Port range updated successfully to %u-%u\n", port_start, port_end);
	return SUCCESS;
}

int clear_host_network_port_range(void)
{
	FILE *fp = NULL;
	char line[256];
	int found = 0;
	char temp_file[] = "/tmp/sysctl.conf.XXXXXX";
	int temp_fd;
	FILE *temp_fp = NULL;

	/* Open sysctl.conf file */
	fp = fopen("/etc/sysctl.conf", "r");
	if (!fp) {
		printf("Failed to open /etc/sysctl.conf: %s\n", strerror(errno));
		return FAILURE;
	}

	/* Create temp file */
	temp_fd = mkstemp(temp_file);
	if (temp_fd < 0) {
		printf("Failed to create temp file: %s\n", strerror(errno));
		fclose(fp);
		return FAILURE;
	}

	temp_fp = fdopen(temp_fd, "w");
	if (!temp_fp) {
		printf("Failed to open temp file: %s\n", strerror(errno));
		close(temp_fd);
		fclose(fp);
		return FAILURE;
	}

	/* Copy all lines except port range setting */
	while (fgets(line, sizeof(line), fp)) {
		if (strstr(line, "net.ipv4.ip_local_port_range")) {
			found = 1;
		} else {
			fputs(line, temp_fp);
		}
	}

	fclose(fp);
	fclose(temp_fp);

	/* Replace original with temp file only if we found and removed an entry */
	if (found) {
		if (rename(temp_file, "/etc/sysctl.conf") != 0) {
			printf("Failed to update sysctl.conf: %s\n", strerror(errno));
			unlink(temp_file);
			return FAILURE;
		}

		/* Apply changes */
		if (system("sysctl -p") != 0) {
			printf("Failed to apply sysctl changes\n");
			return FAILURE;
		}

		printf("Host network port range configuration cleared successfully\n");
	} else {
		/* No entry found, just remove the temp file */
		unlink(temp_file);
		printf("No port range configuration found to clear\n");
	}

	return SUCCESS;
}

