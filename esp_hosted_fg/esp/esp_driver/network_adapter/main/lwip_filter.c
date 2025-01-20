/* LWIP packet filtering implementation */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/etharp.h"
#include "lwip/prot/iana.h"
#include "lwip/prot/ip.h"
#include "lwip/prot/tcp.h"
#include "lwip/prot/udp.h"
#include "lwip/prot/icmp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/priv/tcp_priv.h"

#include "host_power_save.h"
#include "lwip_filter.h"

static const char *TAG = "lwip_filter";

#define MQTT_PORT 1883
#define WAKEUP_HOST_STRING "wakeup-host"
#define DEFAULT_IPERF_PORT 5001

/* Use LWIP's port range macros instead of redefining */
/* #define IS_REMOTE_TCP_PORT(port) ((port) != MQTT_PORT) */
/* #define IS_REMOTE_UDP_PORT(port) (1) */

#ifdef CONFIG_SLAVE_MANAGES_WIFI
  #define DHCP_LWIP_BRIDGE SLAVE_LWIP_BRIDGE
#else
  #define DHCP_LWIP_BRIDGE HOST_LWIP_BRIDGE
#endif

#if defined(CONFIG_ESP_DEFAULT_LWIP_SLAVE)
  #define DEFAULT_LWIP_TO_SEND SLAVE_LWIP_BRIDGE
#elif defined(CONFIG_ESP_DEFAULT_LWIP_HOST)
  #define DEFAULT_LWIP_TO_SEND BOTH_LWIP_BRIDGE
#elif defined(CONFIG_ESP_DEFAULT_LWIP_BOTH)
  #define DEFAULT_LWIP_TO_SEND BOTH_LWIP_BRIDGE
#else
  #error "Select one of the LWIP to forward"
#endif

/* Cache for TCP port checks */
static struct {
	uint64_t last_check_time;
	int last_result;
	uint16_t last_port;
} tcp_cache = {0};

/* Cache for UDP port checks */
static struct {
	uint64_t last_check_time;
	int last_result;
	uint16_t last_port;
} udp_cache = {0};

static bool host_mqtt_wakeup_triggered(const void *payload, uint16_t payload_length)
{
	/* Check if payload contains "wakeup-host" string */
	if (payload_length >= strlen(WAKEUP_HOST_STRING) &&
			memcmp(payload, WAKEUP_HOST_STRING, strlen(WAKEUP_HOST_STRING)) == 0) {
		return true;
	}
	return false;
}

static int is_local_tcp_port_open(uint16_t port)
{
	uint64_t current_time = esp_timer_get_time() >> 10; /* Approx ms */

	/* Return cached result if within 1 sec window */
	if (tcp_cache.last_port == port &&
			(current_time - tcp_cache.last_check_time) < 1000) {
		tcp_cache.last_check_time = current_time;
		return tcp_cache.last_result;
	}

	/* Use LWIP protection for thread safety */
	SYS_ARCH_DECL_PROTECT(old_level);
	SYS_ARCH_PROTECT(old_level);

	int found = 0;
	struct tcp_pcb *pcb;
	for(pcb = tcp_listen_pcbs.pcbs; pcb != NULL; pcb = pcb->next) {
		if (pcb->local_port == port) {
			found = 1;
			break;
		}
	}

	SYS_ARCH_UNPROTECT(old_level);

	/* Cache the result */
	tcp_cache.last_check_time = current_time;
	tcp_cache.last_result = found;
	tcp_cache.last_port = port;

	ESP_LOGI(TAG, "is_local_tcp_port (%u) open: %d", port, found);
	return found;
}

static int is_local_udp_port_open(uint16_t port)
{
	uint64_t current_time = esp_timer_get_time() >> 10; /* Approx ms */

	/* Return cached result if within 1 sec window */
	if (udp_cache.last_port == port &&
			(current_time - udp_cache.last_check_time) < 1000) {
		udp_cache.last_check_time = current_time;
		return udp_cache.last_result;
	}

	/* Use LWIP protection for thread safety */
	SYS_ARCH_DECL_PROTECT(old_level);
	SYS_ARCH_PROTECT(old_level);

	int found = 0;
	struct udp_pcb *pcb;
	for(pcb = udp_pcbs; pcb != NULL; pcb = pcb->next) {
		if (pcb->local_port == port) {
			found = 1;
			break;
		}
	}

	SYS_ARCH_UNPROTECT(old_level);

	/* Cache the result */
	udp_cache.last_check_time = current_time;
	udp_cache.last_result = found;
	udp_cache.last_port = port;

	ESP_LOGI(TAG, "is_local_udp_port (%u) open: %d", port, found);
	return found;
}

hosted_l2_bridge filter_and_route_packet(void *frame_data, uint16_t frame_length)
{
	hosted_l2_bridge result = DEFAULT_LWIP_TO_SEND;

	struct eth_hdr *ethhdr = (struct eth_hdr *)frame_data;
	struct ip_hdr *iphdr;
	u8_t proto;
	u16_t dst_port = 0;
	u16_t src_port = 0;

	/* Check if the frame is a MAC broadcast */
	if (ethhdr->dest.addr[0] & 0x01) {
		result = SLAVE_LWIP_BRIDGE;
		return result;
	}

	/* Check if the frame contains an IP packet */
	if (lwip_ntohs(ethhdr->type) == ETHTYPE_IP) {
		ESP_LOGV(TAG, "new ip packet");

		/* Get the IP header */
		iphdr = (struct ip_hdr *)((u8_t *)frame_data + SIZEOF_ETH_HDR);
		/* Get the protocol from the IP header */
		proto = IPH_PROTO(iphdr);

		if (proto == IP_PROTO_TCP) {
			struct tcp_hdr *tcphdr = (struct tcp_hdr *)((u8_t *)iphdr + IPH_HL(iphdr) * 4);
			dst_port = lwip_ntohs(tcphdr->dest);
			src_port = lwip_ntohs(tcphdr->src);

			ESP_LOGV(TAG, "dst_port: %u, src_port: %u", dst_port, src_port);

			/* Check for iperf port */
			if (dst_port == DEFAULT_IPERF_PORT) {
				ESP_LOGV(TAG, "iperf pkt %u", DEFAULT_IPERF_PORT);
				if (is_local_tcp_port_open(dst_port)) {
					result = SLAVE_LWIP_BRIDGE;
					return result;
				} else if (!is_host_power_saving()) {
					result = HOST_LWIP_BRIDGE;
					return result;
				}
			}

			if (IS_REMOTE_TCP_PORT(dst_port)) {
				if (is_host_power_saving()) {
					/* filter host destined mqtt packet says 'wake-up-host' */
					if (src_port == MQTT_PORT) {
					#define TCP_HDR_LEN(tcphdr) ((TCPH_FLAGS(tcphdr) >> 12) * 4)

						u16_t tcp_hdr_len = TCP_HDR_LEN(tcphdr);
						u16_t mqtt_payload_length = lwip_ntohs(tcphdr->wnd);
						u8_t *mqtt_payload = (u8_t *)tcphdr + tcp_hdr_len;

						if (host_mqtt_wakeup_triggered(mqtt_payload, mqtt_payload_length)) {
							ESP_LOGV(TAG, "Wakeup host: MQTT wakeup pkt");
							result = HOST_LWIP_BRIDGE;
							return result;
						} else {
							/* drop any other host destined mqtt packet */
							result = INVALID_BRIDGE;
							return result;
						}
					} else {
						ESP_LOGV(TAG, "Wakeup host: TCP pkt");
						result = HOST_LWIP_BRIDGE;
						return result;
					}
				} else {
					/* As host is not sleeping, send packets freely */
					result = HOST_LWIP_BRIDGE;
					return result;
				}
			}

		} else if (proto == IP_PROTO_UDP) {
			ESP_LOGV(TAG, "new udp packet");
			struct udp_hdr *udphdr = (struct udp_hdr *)((u8_t *)iphdr + IPH_HL(iphdr) * 4);
			dst_port = lwip_ntohs(udphdr->dest);

			/* Check for iperf UDP port */
			if (dst_port == DEFAULT_IPERF_PORT) {
				ESP_LOGV(TAG, "Detected iperf UDP packet on port %u", DEFAULT_IPERF_PORT);
				if (is_local_udp_port_open(dst_port)) {
					result = SLAVE_LWIP_BRIDGE;
					return result;
				} else if (!is_host_power_saving()) {
					result = HOST_LWIP_BRIDGE;
					return result;
				}
			}

			if (dst_port == LWIP_IANA_PORT_DHCP_CLIENT) {
				result = DHCP_LWIP_BRIDGE;
				return result;
			}

			if (IS_REMOTE_UDP_PORT(dst_port)) {
				ESP_LOGV(TAG, "Wakeup host: UDP pkt");
				result = HOST_LWIP_BRIDGE;
				return result;
			}

		} else if (proto == IP_PROTO_ICMP) {
			ESP_LOGV(TAG, "new icmp packet");
			struct icmp_echo_hdr *icmphdr = (struct icmp_echo_hdr *)((u8_t *)iphdr + IPH_HL(iphdr) * 4);
			if (icmphdr->type == ICMP_ECHO) {
				/* ping request */
				result = SLAVE_LWIP_BRIDGE;
				return result;
			} else if (icmphdr->type == ICMP_ER) {
				if (is_host_power_saving()) {
					result = SLAVE_LWIP_BRIDGE;
					return result;
				} else {
					/* ping response */
					ESP_HEXLOGV("icmp_er", frame_data, frame_length, 64);
					result = BOTH_LWIP_BRIDGE;
					return result;
				}
			}
		}

	} else if (lwip_ntohs(ethhdr->type) == ETHTYPE_ARP) {
		ESP_LOGV(TAG, "new arp packet");
		struct etharp_hdr *arphdr = (struct etharp_hdr *)((u8_t *)frame_data + SIZEOF_ETH_HDR);

		if (arphdr->opcode == lwip_htons(ARP_REQUEST)) {
			result = SLAVE_LWIP_BRIDGE;
			return result;
		} else {
			if (is_host_power_saving()) {
				result = SLAVE_LWIP_BRIDGE;
				return result;
			} else {
				ESP_HEXLOGV("arp_reply", frame_data, frame_length, 64);
				result = BOTH_LWIP_BRIDGE;
				return result;
			}
		}
	}

	return result;
}
