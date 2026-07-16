/* LWIP packet filtering implementation */

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "lwip/netif.h"
#include "esp_private/wifi.h"
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_HOST_PS_READY
#include "eh_cp_feat_host_ps_apis.h"
#endif
#include "eh_cp_feat_nw_split_lwip_hook.h"
#include "eh_cp_feat_nw_split_events.h"
#include "eh_cp_feat_nw_split_apis.h"
#include "eh_cp_feat_wifi.h"
#include "eh_log.h"
#include "eh_common.h"
#include "eh_cp_core.h"           /* EH_CP_FEAT_REGISTER */
#include "eh_cp_event.h"
#include "eh_cp_feat_nw_split.h"

ESP_EVENT_DEFINE_BASE(ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT);

#if EH_CP_FEAT_NW_SPLIT_READY
static esp_err_t nw_split_auto_init(void)
{
    eh_cp_feat_nw_split_config_t cfg = EH_CP_FEAT_NW_SPLIT_CONFIG_INIT_DEFAULT();
    return eh_cp_feat_nw_split_init(&cfg);
}

#if EH_CP_FEAT_NW_SPLIT_AUTO_INIT
EH_CP_FEAT_REGISTER(nw_split_auto_init,
                   eh_cp_feat_nw_split_deinit,
                   "feat_nw_split", tskNO_AFFINITY, 100);
#endif
#else
static esp_err_t nw_split_auto_init(void)
{
    /* nw_split disabled by Kconfig — nothing to do */
    return ESP_OK;
}

#if EH_CP_FEAT_NW_SPLIT_AUTO_INIT
EH_CP_FEAT_REGISTER(nw_split_auto_init,
                   NULL,
                   "feat_nw_split", tskNO_AFFINITY, 100);
#endif
#endif

#if EH_CP_FEAT_NW_SPLIT_READY
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

#include "eh_cp_feat_nw_split.h"
#include "eh_check.h"

static const char *TAG = "ehcp_nw_split";

static eh_cp_feat_nw_split_nw_stack_e g_dhcp_owner;

eh_cp_rx_cb_t orig_wlan_rx_cb_for_host =  NULL;
eh_cp_rx_cb_t nw_split_wlan_sta_rx_cb =	NULL;

/* Coprocessor station netif */
static esp_netif_t *coprocessor_sta_netif = NULL;
static bool s_sta_rx_overridden = false;
static bool s_sta_got_ip = false;
static bool s_sta_netif_input_ready = false;

/* Link-time wrap: suppress STA rxcb overrides while nw_split owns it. */
static bool s_nw_split_owns_sta_rxcb = false;

extern esp_err_t __real_esp_wifi_internal_reg_rxcb(wifi_interface_t ifx, wifi_rxcb_t fn);

esp_err_t __wrap_esp_wifi_internal_reg_rxcb(wifi_interface_t ifx, wifi_rxcb_t fn)
{
	if (ifx == WIFI_IF_STA && s_nw_split_owns_sta_rxcb) {
		ESP_LOGD(TAG, "Suppressing STA rxcb override (%p) — nw_split owns it", fn);
		return ESP_OK;
	}
	return __real_esp_wifi_internal_reg_rxcb(ifx, fn);
}

static void nw_split_core_event_handler(void *handler_args, esp_event_base_t base,
										int32_t event_id, void *event_data)
{
	if (base != EH_CP_EVENT || event_id != EH_CP_EVT_PRIVATE_RPC_READY) {
		return;
	}
	eh_cp_feat_nw_split_replay_status_if_needed();
}

static bool nw_split_netif_input_ready(esp_netif_t *netif)
{
	void *netif_impl = esp_netif_get_netif_impl(netif);
	if (!netif_impl) {
		return false;
	}
	struct netif *lwip_netif = (struct netif *)netif_impl;
	return lwip_netif->input != NULL;
}

static bool nw_split_is_netif_ready_fast(esp_netif_t *netif)
{
	/* Verify lwip_netif->input non-NULL each call — cached flag is racy across reconnect. */
	return nw_split_netif_input_ready(netif);
}

static esp_netif_t *nw_split_get_active_sta_netif(void)
{
	/* Prefer currently-registered default STA netif — avoids stale ptr after WiFi reinit. */
	esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	if (netif) {
		coprocessor_sta_netif = netif;
		return netif;
	}
	return coprocessor_sta_netif;
}
/* Port configuration */
#define MQTT_PORT 1883
#define WAKEUP_HOST_STRING "wakeup-host"
#define DEFAULT_IPERF_PORT 5001
#define MAX_ALLOWED_PORTS 10
#define MAX_PORT_STR_LEN 128

/* Port caching for performance */
#define PORT_CACHE_TIMEOUT_US 1000000 // 1 second

/* Port caching for performance */
static struct {
	uint64_t last_check_time;
	int last_result;
	uint16_t last_port;
} tcp_cache = {0};

static struct {
	uint64_t last_check_time;
	int last_result;
	uint16_t last_port;
} udp_cache = {0};


#if defined(CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_COPROCESSOR)
  #define DEFAULT_NW_STACK_TO_SEND NW_STACK_COPROCESSOR
#elif defined(CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_HOST)
  #define DEFAULT_NW_STACK_TO_SEND NW_STACK_HOST
#elif defined(CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_BOTH)
  #define DEFAULT_NW_STACK_TO_SEND NW_STACK_BOTH
#else
  #error "Select one of the NW_STACK to forward"
#endif

/* Port array definitions */
#define MAX_ALLOWED_TCP_SRC_PORTS 10
#define MAX_ALLOWED_UDP_SRC_PORTS 10
#define MAX_ALLOWED_TCP_DST_PORTS 10
#define MAX_ALLOWED_UDP_DST_PORTS 10

static uint16_t allowed_tcp_src_ports[MAX_ALLOWED_TCP_SRC_PORTS] = {0};
static uint16_t allowed_udp_src_ports[MAX_ALLOWED_UDP_SRC_PORTS] = {0};
static uint16_t allowed_tcp_dst_ports[MAX_ALLOWED_TCP_DST_PORTS] = {0};
static uint16_t allowed_udp_dst_ports[MAX_ALLOWED_UDP_DST_PORTS] = {0};
static int allowed_tcp_src_ports_count = 0;
static int allowed_udp_src_ports_count = 0;
static int allowed_tcp_dst_ports_count = 0;
static int allowed_udp_dst_ports_count = 0;

static esp_err_t nw_split_get_status(uint8_t iface, eh_cp_feat_nw_split_status_t *status);
static esp_err_t nw_split_set_config(uint8_t iface, const char *ip, const char *nm,
	const char *gw, const char *dns_ip, uint8_t dns_type);


/* Parse a comma-separated list of ports into an array - using old code logic */
static int init_allowed_ports(const char *ports_str, int *ports_count, uint16_t *ports_array, int max_ports, const char *port_type) {
	/* Only initialize once */
	static bool tcp_src_initialized = false;
	static bool tcp_dst_initialized = false;
	static bool udp_src_initialized = false;
	static bool udp_dst_initialized = false;

	/* Check if already initialized for this port type */
	if ((strcmp(port_type, "tcp_src") == 0 && tcp_src_initialized) ||
		(strcmp(port_type, "tcp_dst") == 0 && tcp_dst_initialized) ||
		(strcmp(port_type, "udp_src") == 0 && udp_src_initialized) ||
		(strcmp(port_type, "udp_dst") == 0 && udp_dst_initialized)) {
		return 0;
	}

	/* Reset counter */
	*ports_count = 0;

	/* If no ports string provided, return */
	if (!ports_str || !ports_str[0]) {
		ESP_LOGI(TAG, "No %s ports configured", port_type);
		return 0;
	}

	char port_buf[6]; /* Max 5 digits for a port + null terminator */
	int port_buf_idx = 0;

	for (int i = 0; ports_str[i] != '\0' && *ports_count < max_ports; i++) {
		if (isdigit((unsigned char)ports_str[i])) {
			port_buf[port_buf_idx++] = ports_str[i];
			if (port_buf_idx >= sizeof(port_buf) - 1) {
				port_buf_idx = sizeof(port_buf) - 2; /* Prevent overflow */
			}
		} else if (ports_str[i] == ',') {
			if (port_buf_idx > 0) {
				port_buf[port_buf_idx] = '\0';
				ports_array[(*ports_count)++] = atoi(port_buf);
				port_buf_idx = 0;
			}
		}
	}

	/* Process the last port if there's no trailing comma */
	if (port_buf_idx > 0) {
		port_buf[port_buf_idx] = '\0';
		ports_array[(*ports_count)++] = atoi(port_buf);
	}

	/* Log the results */
	ESP_LOGI(TAG, "Initialized %d allowed %s ports:", *ports_count, port_type);
	for (int i = 0; i < *ports_count; i++) {
		ESP_LOGI(TAG, "  - Port %d", ports_array[i]);
	}

	/* Mark as initialized */
	if (strcmp(port_type, "tcp_src") == 0) {
		tcp_src_initialized = true;
	} else if (strcmp(port_type, "tcp_dst") == 0) {
		tcp_dst_initialized = true;
	} else if (strcmp(port_type, "udp_src") == 0) {
		udp_src_initialized = true;
	} else if (strcmp(port_type, "udp_dst") == 0) {
		udp_dst_initialized = true;
	}

	return 0;
}

/* Initialize TCP port forwarding rules */
static int init_allowed_tcp_ports(const char *ports_str_src, const char *ports_str_dst) {
	int ret1 = 0, ret2 = 0;

	if (ports_str_src && strlen(ports_str_src) > 0) {
		ESP_LOGI(TAG, "Host reserved TCP src ports: %s", ports_str_src);
		ret1 = init_allowed_ports(ports_str_src, &allowed_tcp_src_ports_count,
								 allowed_tcp_src_ports, MAX_ALLOWED_TCP_SRC_PORTS, "tcp_src");
	}

	if (ports_str_dst && strlen(ports_str_dst) > 0) {
		ESP_LOGI(TAG, "Host reserved TCP dst ports: %s", ports_str_dst);
		ret2 = init_allowed_ports(ports_str_dst, &allowed_tcp_dst_ports_count,
								 allowed_tcp_dst_ports, MAX_ALLOWED_TCP_DST_PORTS, "tcp_dst");
	}

	if (ret1) {
		ESP_LOGE(TAG, "Failed to initialize allowed TCP src ports");
		return ret1;
	}
	if (ret2) {
		ESP_LOGE(TAG, "Failed to initialize allowed TCP dst ports");
		return ret2;
	}
	return 0;
}

/* Initialize UDP port forwarding rules */
static int init_allowed_udp_ports(const char *ports_str_src, const char *ports_str_dst) {
	int ret1 = 0, ret2 = 0;

	if (ports_str_src && strlen(ports_str_src) > 0) {
		ESP_LOGI(TAG, "host reserved udp src ports: %s", ports_str_src);
		ret1 = init_allowed_ports(ports_str_src, &allowed_udp_src_ports_count,
				allowed_udp_src_ports, MAX_ALLOWED_UDP_SRC_PORTS, "udp_src");
	}
	if (ports_str_dst && strlen(ports_str_dst) > 0) {
		ESP_LOGI(TAG, "host reserved udp dst ports: %s", ports_str_dst);
		ret2 = init_allowed_ports(ports_str_dst, &allowed_udp_dst_ports_count,
				allowed_udp_dst_ports, MAX_ALLOWED_UDP_DST_PORTS, "udp_dst");
	}

	if (ret1) {
		ESP_LOGE(TAG, "Failed to initialize allowed udp src ports");
		return ret1;
	}
	if (ret2) {
		ESP_LOGE(TAG, "Failed to initialize allowed udp dst ports");
		return ret2;
	}
	return 0;
}

/* Configure host static port forwarding rules */
int eh_cp_feat_nw_split_config_host_static_port_forwarding_rules(
	const char *ports_str_tcp_src,
	const char *ports_str_tcp_dst,
	const char *ports_str_udp_src,
	const char *ports_str_udp_dst) {

	int ret1 = 0, ret2 = 0;

	ESP_LOGI(TAG, "Host reserved TCP src ports: %s",
			 ports_str_tcp_src && strlen(ports_str_tcp_src) > 0 ? ports_str_tcp_src : "none");
	ESP_LOGI(TAG, "Host reserved TCP dst ports: %s",
			 ports_str_tcp_dst && strlen(ports_str_tcp_dst) > 0 ? ports_str_tcp_dst : "none");
	ret1 = init_allowed_tcp_ports(ports_str_tcp_src, ports_str_tcp_dst);

	ESP_LOGI(TAG, "Host reserved UDP src ports: %s",
			 ports_str_udp_src && strlen(ports_str_udp_src) > 0 ? ports_str_udp_src : "none");
	ESP_LOGI(TAG, "Host reserved UDP dst ports: %s",
			 ports_str_udp_dst && strlen(ports_str_udp_dst) > 0 ? ports_str_udp_dst : "none");
	ret2 = init_allowed_udp_ports(ports_str_udp_src, ports_str_udp_dst);

	if (ret1) {
		ESP_LOGE(TAG, "Failed to initialize allowed TCP ports");
		return ret1;
	}
	if (ret2) {
		ESP_LOGE(TAG, "Failed to initialize allowed UDP ports");
		return ret2;
	}
	return 0;
}

/* Port checking functions */
static inline bool is_tcp_src_port_allowed(uint16_t port) {
	for (int i = 0; i < allowed_tcp_src_ports_count; i++) {
		if (port == allowed_tcp_src_ports[i]) {
			return true;
		}
	}
	return false;
}

static inline bool is_tcp_dst_port_allowed(uint16_t port) {
	for (int i = 0; i < allowed_tcp_dst_ports_count; i++) {
		if (port == allowed_tcp_dst_ports[i]) {
			return true;
		}
	}
	return false;
}

static inline bool is_udp_src_port_allowed(uint16_t port) {
	for (int i = 0; i < allowed_udp_src_ports_count; i++) {
		if (port == allowed_udp_src_ports[i]) {
			return true;
		}
	}
	return false;
}

static inline bool is_udp_dst_port_allowed(uint16_t port) {
	for (int i = 0; i < allowed_udp_dst_ports_count; i++) {
		if (port == allowed_udp_dst_ports[i]) {
			return true;
		}
	}
	return false;
}

/* Check if the packet is an MQTT wakeup packet */
static bool host_mqtt_wakeup_triggered(const void *payload, uint16_t payload_length)
{
	/* Check if payload contains "wakeup-host" string */
	if (payload_length >= strlen(WAKEUP_HOST_STRING) &&
			memcmp(payload, WAKEUP_HOST_STRING, strlen(WAKEUP_HOST_STRING)) == 0) {
		return true;
	}
	return false;
}

/* Check if a TCP port is open locally - using old code logic with thread safety */
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

/* Check if a UDP port is open locally - using old code logic with thread safety */
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

/* Filter and route packet based on port number and power saving state */
eh_cp_feat_nw_split_nw_stack_e eh_cp_feat_nw_split_filter_packet(void *frame_data, uint16_t frame_length)
{
	if (!frame_data || frame_length < SIZEOF_ETH_HDR) {
		return NW_STACK_INVALID;
	}

	eh_cp_feat_nw_split_nw_stack_e result = DEFAULT_NW_STACK_TO_SEND;
	struct eth_hdr *ethhdr = (struct eth_hdr *)frame_data;
	struct ip_hdr *iphdr;
	u8_t proto;
	u16_t dst_port = 0;
	u16_t src_port = 0;
	uint8_t is_host_power_saving = 0;

	/* C9: direct call — no ops registry */
    #if EH_CP_FEAT_HOST_PS_READY
    is_host_power_saving = (uint8_t)eh_cp_feat_host_ps_is_host_power_saving();
    #else
    is_host_power_saving = 0;
    #endif

	/* Check if the frame is a MAC broadcast */
	if (ethhdr->dest.addr[0] & 0x01) {
		result = NW_STACK_COPROCESSOR;
		return result;
	}

	/* Check if it's an IP packet */
	if (ethhdr->type == PP_HTONS(ETHTYPE_IP)) {
		/* Ensure we have enough data for IP header */
		if (frame_length < (SIZEOF_ETH_HDR + IP_HLEN)) {
			return NW_STACK_INVALID;
		}

		iphdr = (struct ip_hdr *)((u8_t *)frame_data + SIZEOF_ETH_HDR);
		proto = IPH_PROTO(iphdr);

		/* Check for TCP packet */
		if (proto == IP_PROTO_TCP) {
			/* Ensure we have enough data for TCP header */
			if (frame_length < (SIZEOF_ETH_HDR + IPH_HL(iphdr) * 4 + TCP_HLEN)) {
				return NW_STACK_INVALID;
			}

			struct tcp_hdr *tcphdr = (struct tcp_hdr *)((u8_t *)iphdr + IPH_HL(iphdr) * 4);
			src_port = lwip_ntohs(tcphdr->src);
			dst_port = lwip_ntohs(tcphdr->dest);

			ESP_LOGV(TAG, "TCP dst_port: %u, src_port: %u", dst_port, src_port);

			/* Check if TCP port is in the allowed list for host */
			if (is_tcp_src_port_allowed(src_port) || is_tcp_dst_port_allowed(dst_port)) {
				ESP_LOGV(TAG, "Priority tcp port traffic detected, forwarding to host");
				result = NW_STACK_HOST;
				return result;
			}

			/* Check for iperf port */
			else if (dst_port == DEFAULT_IPERF_PORT) {
				ESP_LOGV(TAG, "iperf pkt %u", DEFAULT_IPERF_PORT);
				if (is_local_tcp_port_open(dst_port)) {
					result = NW_STACK_COPROCESSOR;
					return result;
				} else if (!is_host_power_saving) {
					result = NW_STACK_HOST;
					return result;
				}
			}

			/* Check if TCP port is in the remote port range */
			else if (IS_REMOTE_TCP_PORT(dst_port)) {
				if (is_host_power_saving) {
					/* Special handling for MQTT packets that should wake up the host */
					if (src_port == MQTT_PORT) {
						#define TCP_HDR_LEN(tcphdr) ((TCPH_FLAGS(tcphdr) >> 12) * 4)

						u16_t tcp_hdr_len = TCP_HDR_LEN(tcphdr);
						u16_t mqtt_payload_length = lwip_ntohs(tcphdr->wnd);
						u8_t *mqtt_payload = (u8_t *)tcphdr + tcp_hdr_len;

						if (host_mqtt_wakeup_triggered(mqtt_payload, mqtt_payload_length)) {
							ESP_LOGV(TAG, "Wakeup host: MQTT wakeup pkt");
							result = NW_STACK_HOST;
							return result;
						} else {
							/* drop any other host destined mqtt packet */
							result = NW_STACK_INVALID;
							ESP_LOGW(TAG, "mqtt pkt DROPPED dst %u src %u => lwip %u", dst_port, src_port, result);
							return result;
						}
					} else {
						ESP_LOGV(TAG, "Wakeup host: TCP pkt");
						result = NW_STACK_INVALID;
						ESP_LOGW(TAG, "host pkt dropped in power save (dst %u src %u)", dst_port, src_port);
						return result;
					}
				} else {
					/* As host is not sleeping, send packets freely */
					ESP_LOGV(TAG, "mqtt pkt -> host. src %u dst %u", src_port, dst_port);
					result = NW_STACK_HOST;
					return result;
				}
			}
			/* Check if TCP port is in the local port range */
			else if (IS_LOCAL_TCP_PORT(dst_port)) {
				result = NW_STACK_COPROCESSOR;
				ESP_LOGV(TAG, "mqtt pkt -> coprocessor. src %u dst %u", src_port, dst_port);
				return result;
			}

			return DEFAULT_NW_STACK_TO_SEND;
		}
		/* Check for UDP packet */
		else if (proto == IP_PROTO_UDP) {
			/* Ensure we have enough data for UDP header */
			if (frame_length < (SIZEOF_ETH_HDR + IPH_HL(iphdr) * 4 + UDP_HLEN)) {
				return NW_STACK_INVALID;
			}

			struct udp_hdr *udphdr = (struct udp_hdr *)((u8_t *)iphdr + IPH_HL(iphdr) * 4);
			src_port = lwip_ntohs(udphdr->src);
			dst_port = lwip_ntohs(udphdr->dest);

			ESP_LOGV(TAG, "UDP dst_port: %u, src_port: %u", dst_port, src_port);

			/* Check if UDP port is in the allowed list for host */
			if (is_udp_src_port_allowed(src_port) || is_udp_dst_port_allowed(dst_port)) {
				ESP_LOGV(TAG, "Priority udp port traffic detected, forwarding to host");
				result = NW_STACK_HOST;
				return result;
			}

			/* Check for iperf UDP port */
			else if (dst_port == DEFAULT_IPERF_PORT) {
				ESP_LOGV(TAG, "Detected iperf UDP packet on port %u", DEFAULT_IPERF_PORT);
				if (is_local_udp_port_open(dst_port)) {
					result = NW_STACK_COPROCESSOR;
					return result;
				} else if (!is_host_power_saving) {
					result = NW_STACK_HOST;
					return result;
				}
			}

			/* Special case for DHCP client port */
			else if (dst_port == LWIP_IANA_PORT_DHCP_CLIENT) {
				result = g_dhcp_owner;
				return result;
			}

			/* Check if UDP port is in the remote port range */
			else if (IS_REMOTE_UDP_PORT(dst_port)) {
				if (is_host_power_saving) {
					ESP_LOGW(TAG, "host pkt dropped in power save (dst %u src %u)", dst_port, src_port);
					result = NW_STACK_INVALID;
					return result;
				} else {
					result = NW_STACK_HOST;
					return result;
				}
			}
			/* Check if UDP port is in the local port range */
			else if (IS_LOCAL_UDP_PORT(dst_port)) {
				result = NW_STACK_COPROCESSOR;
				return result;
			}

			return DEFAULT_NW_STACK_TO_SEND;
		}
		/* Check for ICMP packet */
		else if (proto == IP_PROTO_ICMP) {
			/* Ensure we have enough data for ICMP header */
			/*if (frame_length < (SIZEOF_ETH_HDR + IPH_HL(iphdr) * 4 + ICMP_HLEN)) {
				return NW_STACK_INVALID;
			}*/

			ESP_LOGV(TAG, "new icmp packet");
			struct icmp_echo_hdr *icmphdr = (struct icmp_echo_hdr *)((u8_t *)iphdr + IPH_HL(iphdr) * 4);

			/* Handle echo requests (ping) */
			if (ICMPH_TYPE(icmphdr) == ICMP_ECHO) {
				/* ping request */
				result = NW_STACK_COPROCESSOR;
				return result;
			}
			/* Handle echo replies (ping response) */
			else if (ICMPH_TYPE(icmphdr) == ICMP_ER) {
				if (is_host_power_saving) {
					result = NW_STACK_COPROCESSOR;
					return result;
				} else {
					/* ping response */
					ESP_HEXLOGV("icmp_er", frame_data, frame_length, 64);
					result = NW_STACK_BOTH;
					return result;
				}
			}

			return DEFAULT_NW_STACK_TO_SEND;
		}
	}
	/* Check if it's an ARP packet */
	else if (ethhdr->type == PP_HTONS(ETHTYPE_ARP)) {
		/* Ensure we have enough data for ARP header */
		if (frame_length < (SIZEOF_ETH_HDR + SIZEOF_ETHARP_HDR)) {
			return NW_STACK_INVALID;
		}

		ESP_LOGV(TAG, "new arp packet");
		struct etharp_hdr *arphdr = (struct etharp_hdr *)((u8_t *)frame_data + SIZEOF_ETH_HDR);

		/* Handle ARP requests */
		if (arphdr->opcode == PP_HTONS(ARP_REQUEST)) {
			result = NW_STACK_COPROCESSOR;
			return result;
		}
		/* Handle ARP replies */
		else {
			if (is_host_power_saving) {
				result = NW_STACK_COPROCESSOR;
				return result;
			} else {
				ESP_HEXLOGV("arp_reply", frame_data, frame_length, 64);
				result = NW_STACK_BOTH;
				return result;
			}
		}
	}

	/* Default routing for other packet types */
	return result;
}

IRAM_ATTR esp_err_t eh_cp_feat_nw_split_wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
	eh_cp_feat_nw_split_nw_stack_e stack_to_send = NW_STACK_HOST;
	void (*wifi_free_func)(void *data) = esp_wifi_internal_free_rx_buffer;
	assert(orig_wlan_rx_cb_for_host);

	if (!buffer || !eb) {
		if (eb && wifi_free_func) {
			ESP_LOGD(TAG, "drop wifi packet");
			wifi_free_func(eb);
		}
		return ESP_OK;
	}

	if (!orig_wlan_rx_cb_for_host) {
		ESP_LOGW(TAG, "orig_wlan_rx_cb_for_host is NULL, drop packet");
		if (eb && wifi_free_func) {
			wifi_free_func(eb);
		}
		return ESP_OK;
	}

	/* Filter and route the packet based on destination port */
	stack_to_send = eh_cp_feat_nw_split_filter_packet(buffer, len);

	switch (stack_to_send) {
		case NW_STACK_HOST:
			/* Use the default callback to send to host */
			ESP_LOGV(TAG, "Routing packet to HOST");
			return orig_wlan_rx_cb_for_host(buffer, len, eb);

		case NW_STACK_COPROCESSOR: {
			/* Send to local LWIP */
			ESP_LOGV(TAG, "Routing packet to COPROCESSOR");
			esp_netif_t *active_netif = nw_split_get_active_sta_netif();
			if (!active_netif || !esp_netif_is_netif_up(active_netif)
			    || !nw_split_is_netif_ready_fast(active_netif)) {
				ESP_LOGD(TAG, "coprocessor netif not ready, route to HOST");
				return orig_wlan_rx_cb_for_host(buffer, len, eb);
			}
			esp_netif_receive(active_netif, buffer, len, eb);
			break;
		}

		case NW_STACK_BOTH: {
			ESP_LOGV(TAG, "Routing packet to BOTH coprocessor & host");
			esp_netif_t *active_netif = nw_split_get_active_sta_netif();
			if (!active_netif || !esp_netif_is_netif_up(active_netif)) {
				ESP_LOGD(TAG, "coprocessor netif not ready, route to HOST only");
				return orig_wlan_rx_cb_for_host(buffer, len, eb);
			}

			/* Make a copy for the host */
			void *copy_buff = malloc(len);
			if (!copy_buff) {
				ESP_LOGE(TAG, "Failed to allocate memory for packet copy");
				if (eb && wifi_free_func) {
					wifi_free_func(eb);
				}
				return ESP_OK;
			}
			memcpy(copy_buff, buffer, len);

			/* Coprocessor LWIP gets original buffer + eb */
			if (!esp_netif_is_netif_up(active_netif) || !nw_split_is_netif_ready_fast(active_netif)) {
				ESP_LOGD(TAG, "coprocessor netif not ready, drop coprocessor part");
				if (eb && wifi_free_func) {
					wifi_free_func(eb);
				}
			} else {
			esp_netif_receive(active_netif, buffer, len, eb);
			/* netif frees eb after processing */
			}

			/* Host gets copy via orig callback */
			orig_wlan_rx_cb_for_host(copy_buff, len, NULL);
			break;
		}

		default:
			ESP_LOGD(TAG, "Packet filtering failed, drop packet");
			if (eb && wifi_free_func) {
				wifi_free_func(eb);
			}
			break;
	}

	return ESP_OK;
}

static void populate_dhcp_owner(eh_cp_feat_nw_split_nw_stack_e dhcp_owner)
{
	g_dhcp_owner = dhcp_owner;
	switch (g_dhcp_owner) {
		case NW_STACK_COPROCESSOR:
			ESP_LOGI(TAG, "DHCP at coprocessor");
			g_dhcp_owner = NW_STACK_COPROCESSOR;
			break;
		case NW_STACK_HOST:
			ESP_LOGI(TAG, "DHCP at host");
			g_dhcp_owner = NW_STACK_HOST;
			break;
		case NW_STACK_BOTH:
			ESP_LOGI(TAG, "DHCP at both");
			g_dhcp_owner = NW_STACK_BOTH;
			break;
		default:
			ESP_LOGI(TAG, "DHCP owner invalid");
			g_dhcp_owner = NW_STACK_INVALID;
			ESP_LOGE(TAG, "Invalid DHCP owner");
			break;
	}
}

static void nw_split_send_status_event(void);
static void nw_split_try_override_sta_rx_cb(void);

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
							  int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT) {
		switch (event_id) {
			case WIFI_EVENT_STA_START:
				ESP_LOGD(TAG, "Wi-Fi station started");
#if EH_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START
				if (!eh_cp_feat_wifi_is_station_connecting() &&
				    !eh_cp_feat_wifi_is_connect_pending_on_start()) {
					ESP_LOGW(TAG, "Triggering auto connect on sta start");
					eh_cp_feat_wifi_request_connect();
				}
#endif
				break;

			case WIFI_EVENT_STA_CONNECTED:
				ESP_LOGD(TAG, "Wi-Fi connected");
				nw_split_try_override_sta_rx_cb();
				break;

			case WIFI_EVENT_STA_DISCONNECTED:
				ESP_LOGD(TAG, "Wi-Fi disconnected");
				s_sta_got_ip = false;
				s_sta_rx_overridden = false;
				s_sta_netif_input_ready = false;
				eh_cp_rx_register(ESP_STA_IF, orig_wlan_rx_cb_for_host);
				nw_split_send_status_event();
				break;

			default:
				break;
		}
	} else if (event_base == IP_EVENT) {
		switch (event_id) {
			case IP_EVENT_STA_GOT_IP: {
				nw_split_send_status_event();
				s_sta_got_ip = true;
				nw_split_try_override_sta_rx_cb();
				break;
			}


			default:
				break;
		}
	}
}

static void nw_split_send_status_event(void)
{
	eh_cp_feat_nw_split_status_t status = {0};
	esp_err_t ret = ESP_OK;

	/* Get current network status */
	ret = nw_split_get_status(WIFI_IF_STA, &status);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get network status: %d", ret);
		return;
	}

	/* Send event using standard ESP event API */
	eh_cp_feat_nw_split_evt_t event_id = status.net_link_up ?
							   ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_UP :
							   ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_DOWN;

	ESP_LOGI(TAG, "Sending network %s event: iface=%d, IP=%s",
			 status.net_link_up ? "UP" : "DOWN",
			 status.iface,
			 status.dhcp_ip);

	ret = esp_event_post(ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT,
						event_id,
						&status,
						sizeof(eh_cp_feat_nw_split_status_t),
						portMAX_DELAY);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to post network event: %d", ret);
	}
}

void eh_cp_feat_nw_split_replay_status_if_needed(void)
{
	eh_cp_feat_nw_split_status_t status = {0};

	if (nw_split_get_status(WIFI_IF_STA, &status) != ESP_OK) {
		return;
	}

	/* Replay only when DHCP/IP is already assigned */
	if (status.dhcp_up) {
		nw_split_send_status_event();
	}
}

static void nw_split_try_override_sta_rx_cb(void)
{
	if (s_sta_rx_overridden) {
		return;
	}
	esp_netif_t *active_netif = nw_split_get_active_sta_netif();
	if (!active_netif || !esp_netif_is_netif_up(active_netif)) {
		return;
	}

	void *netif_impl = esp_netif_get_netif_impl(active_netif);
	if (!netif_impl) {
		return;
	}

	s_nw_split_owns_sta_rxcb = false;  /* temporarily allow the registration */
	eh_cp_rx_register(ESP_STA_IF, nw_split_wlan_sta_rx_cb);
	s_nw_split_owns_sta_rxcb = true;   /* lock it — suppress all future overrides */
	s_sta_rx_overridden = true;
}


static esp_err_t nw_split_get_status(uint8_t iface, eh_cp_feat_nw_split_status_t *status)
{
	if (!status) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	esp_netif_t *netif = NULL;
	if (iface == WIFI_IF_STA) {
		netif = coprocessor_sta_netif ? coprocessor_sta_netif : esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	} else if (iface == WIFI_IF_AP) {
		netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
	} else {
		ESP_LOGE(TAG, "Unsupported interface: %u", iface);
		return ESP_FAIL;
	}

	if (!netif) {
		ESP_LOGE(TAG, "Network interface not available");
		return ESP_FAIL;
	}

	/* Fill status structure */
	status->iface = iface;
	status->net_link_up = esp_netif_is_netif_up(netif) ? 1 : 0;

	/* Get IP info */
	esp_netif_ip_info_t ip_info = {0};
	if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
		status->dhcp_up = (ip_info.ip.addr != 0) ? 1 : 0;
		snprintf(status->dhcp_ip, sizeof(status->dhcp_ip), IPSTR, IP2STR(&ip_info.ip));
		snprintf(status->dhcp_nm, sizeof(status->dhcp_nm), IPSTR, IP2STR(&ip_info.netmask));
		snprintf(status->dhcp_gw, sizeof(status->dhcp_gw), IPSTR, IP2STR(&ip_info.gw));
	}

	/* Get DNS info */
	esp_netif_dns_info_t dns_info = {0};
	if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK) {
		status->dns_up = (dns_info.ip.u_addr.ip4.addr != 0) ? 1 : 0;
		snprintf(status->dns_ip, sizeof(status->dns_ip), IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
		status->dns_type = ESP_NETIF_DNS_MAIN;
	}

	ESP_LOGD(TAG, "Network status: iface=%u, link_up=%u, dhcp_up=%u, dns_up=%u",
			 status->iface, status->net_link_up, status->dhcp_up, status->dns_up);
	ESP_LOGD(TAG, "IP: %s, NM: %s, GW: %s, DNS: %s",
			 status->dhcp_ip, status->dhcp_nm, status->dhcp_gw, status->dns_ip);

	return ESP_OK;
}


static esp_err_t nw_split_set_config(uint8_t iface, const char *ip, const char *nm,
									 const char *gw, const char *dns_ip, uint8_t dns_type)
{
	if (!ip || !nm || !gw || !dns_ip) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	esp_netif_t *netif = NULL;
	if (iface == WIFI_IF_STA) {
		netif = coprocessor_sta_netif ? coprocessor_sta_netif : esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	} else if (iface == WIFI_IF_AP) {
		netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
	} else {
		ESP_LOGE(TAG, "Unsupported interface: %u", iface);
		return ESP_FAIL;
	}

	if (!netif) {
		ESP_LOGE(TAG, "Network interface not available");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Setting network config: iface=%u, IP=%s, NM=%s, GW=%s, DNS=%s",
			 iface, ip, nm, gw, dns_ip);

	/* Set IP configuration */
	esp_netif_ip_info_t ip_info = {0};
	ip_info.ip.addr = esp_ip4addr_aton(ip);
	ip_info.netmask.addr = esp_ip4addr_aton(nm);
	ip_info.gw.addr = esp_ip4addr_aton(gw);

	esp_err_t ret = esp_netif_set_ip_info(netif, &ip_info);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set IP info: %s", esp_err_to_name(ret));
		return ret;
	}

	/* Set DNS configuration */
	esp_netif_dns_info_t dns_info = {0};
	dns_info.ip.u_addr.ip4.addr = esp_ip4addr_aton(dns_ip);
	dns_info.ip.type = ESP_IPADDR_TYPE_V4;

	ret = esp_netif_set_dns_info(netif, (esp_netif_dns_type_t)dns_type, &dns_info);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set DNS info: %s", esp_err_to_name(ret));
		return ret;
	}

	ESP_LOGI(TAG, "Network configuration set successfully");
	return ESP_OK;
}

static esp_netif_t *eh_cp_feat_nw_split_init_coprocessor_netif(eh_cp_feat_nw_split_config_t *config)
{
	if (coprocessor_sta_netif) {
		ESP_LOGD(TAG, "Coprocessor netif already exists, reusing");
		return coprocessor_sta_netif;
	}

	/* Reuse app-created WIFI_STA_DEF if present — else netif->input NULL crash. */
	esp_netif_t *existing = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	if (existing) {
		ESP_LOGI(TAG, "Reusing existing WIFI_STA_DEF netif");
		coprocessor_sta_netif = existing;
		return coprocessor_sta_netif;
	}

	bool dhcp_at_coprocessor = false;

	if (config->default_nw_stack_dhcp == NW_STACK_COPROCESSOR ||
	    config->default_nw_stack_dhcp == NW_STACK_BOTH) {
		ESP_LOGI(TAG, "DHCP at coprocessor");
		dhcp_at_coprocessor = true;
	} else {
		ESP_LOGI(TAG, "No DHCP at coprocessor");
	}

	esp_netif_inherent_config_t netif_cfg;
	memcpy(&netif_cfg, ESP_NETIF_BASE_DEFAULT_WIFI_STA, sizeof(netif_cfg));

	if (!dhcp_at_coprocessor)
		netif_cfg.flags &= ~ESP_NETIF_DHCP_CLIENT;

	esp_netif_config_t cfg_sta = {
		.base = &netif_cfg,
		.stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA,
	};
	esp_netif_t *netif_sta = esp_netif_new(&cfg_sta);
	assert(netif_sta);

	EH_CHECK_OK(esp_netif_attach_wifi_station(netif_sta));
	EH_CHECK_OK(esp_wifi_set_default_wifi_sta_handlers());

	if (!dhcp_at_coprocessor) {
		EH_CHECK_OK(esp_netif_dhcpc_stop(netif_sta));
	}
	/* DHCP client auto-starts when netif is created with ESP_NETIF_DHCP_CLIENT. */

	coprocessor_sta_netif = netif_sta;
	assert(coprocessor_sta_netif);

	/* netif start happens in WIFI_EVENT_STA_START default handler — don't start here. */

	if (coprocessor_sta_netif) {
		esp_netif_ip_info_t ip_info = {0};
		if (esp_netif_get_ip_info(coprocessor_sta_netif, &ip_info) == ESP_OK) {
			ESP_LOGD(TAG, "Coprocessor IP: " IPSTR, IP2STR(&ip_info.ip));
			ESP_LOGD(TAG, "Coprocessor NM: " IPSTR, IP2STR(&ip_info.netmask));
			ESP_LOGD(TAG, "Coprocessor GW: " IPSTR, IP2STR(&ip_info.gw));
		}

		ESP_LOGI(TAG, "Netif is up: %s", esp_netif_is_netif_up(coprocessor_sta_netif) ? "YES" : "NO");
	}
	ESP_LOGD(TAG, "========================");

	return netif_sta;
}


/* Initialize the network split functionality */
esp_err_t eh_cp_feat_nw_split_init(eh_cp_feat_nw_split_config_t *config)
{
	assert(config);
	assert(config->wlan_sta_rx_callback_for_coprocessor);

	esp_log_level_set("esp_netif_lwip", ESP_LOG_DEBUG);
	esp_log_level_set("esp_netif_handlers", ESP_LOG_DEBUG);

	/* Ensure LwIP/netif core is initialized before we create or attach netifs.
	 * The app may call esp_netif_init() later; this is safe to call twice. */
	EH_CHECK_OK(esp_netif_init());

	orig_wlan_rx_cb_for_host = eh_cp_rx_get(ESP_STA_IF);
	if (!orig_wlan_rx_cb_for_host) {
		ESP_LOGE(TAG, "Failed to get default hosted wlan rx callback. have you called eh_cp_init() first?");
		return ESP_FAIL;
	}

	/* Configure default port forwarding rules */
#ifdef CONFIG_ESP_HOSTED_HOST_ONLY_PORTS_CONFIGURED
	eh_cp_feat_nw_split_config_host_static_port_forwarding_rules(
		config->host_reserved_ports.tcp_src_ports,
		config->host_reserved_ports.tcp_dst_ports,
		config->host_reserved_ports.udp_src_ports,
		config->host_reserved_ports.udp_dst_ports
	);
#endif

	populate_dhcp_owner(config->default_nw_stack_dhcp);

	/* Create coprocessor netif for station mode */
	coprocessor_sta_netif = eh_cp_feat_nw_split_init_coprocessor_netif(config);
	if (!coprocessor_sta_netif) {
		ESP_LOGE(TAG, "Failed to create coprocessor netif");
		return ESP_FAIL;
	}

	/* Register event handlers */
	EH_CHECK_OK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
		wifi_event_handler, NULL));
	EH_CHECK_OK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
		wifi_event_handler, NULL));
	EH_CHECK_OK(esp_event_handler_register(EH_CP_EVENT,
		EH_CP_EVT_PRIVATE_RPC_READY, nw_split_core_event_handler, NULL));


	nw_split_wlan_sta_rx_cb = config->wlan_sta_rx_callback_for_coprocessor;

	/* Replay missed lifecycle if Wi-Fi already started before nw_split init. */
	if (coprocessor_sta_netif && eh_cp_feat_wifi_is_started()) {
		if (!esp_netif_is_netif_up(coprocessor_sta_netif)) {
			esp_netif_action_start(coprocessor_sta_netif,
				WIFI_EVENT, WIFI_EVENT_STA_START, NULL);
		}
		if (eh_cp_feat_wifi_is_station_connected()) {
			esp_netif_dhcp_status_t dhcpc_status = ESP_NETIF_DHCP_INIT;
			if (esp_netif_dhcpc_get_status(coprocessor_sta_netif, &dhcpc_status) == ESP_OK &&
			    dhcpc_status != ESP_NETIF_DHCP_STARTED) {
				esp_netif_action_connected(coprocessor_sta_netif,
					WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, NULL);
			}
		}
	}

	/* Lock STA rxcb ownership — wrap now suppresses esp_wifi_internal_reg_rxcb(STA,...). */
	s_nw_split_owns_sta_rxcb = true;

	return ESP_OK;
}

esp_err_t eh_cp_feat_nw_split_deinit(void)
{
	ESP_LOGI(TAG, "Deinitializing network split component");

	/* Unsubscription is handled by each RPC adapter in its own deinit(). */

	/* Unregister event handlers */
	EH_CHECK_OK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
											   wifi_event_handler));
	EH_CHECK_OK(esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID,
											   wifi_event_handler));
	EH_CHECK_OK(esp_event_handler_unregister(EH_CP_EVENT,
		EH_CP_EVT_PRIVATE_RPC_READY, nw_split_core_event_handler));
	s_nw_split_owns_sta_rxcb = false;  /* allow restore */
	eh_cp_rx_register(ESP_STA_IF, orig_wlan_rx_cb_for_host);

	ESP_LOGI(TAG, "Network split deinitialization completed");
	return ESP_OK;
}

void eh_cp_feat_nw_split_reset_rx_override(void)
{
	s_sta_got_ip = false;
	s_sta_rx_overridden = false;
	s_sta_netif_input_ready = false;
	s_nw_split_owns_sta_rxcb = false;  /* allow restore */
	coprocessor_sta_netif = NULL;
	if (orig_wlan_rx_cb_for_host) {
		eh_cp_rx_register(ESP_STA_IF, orig_wlan_rx_cb_for_host);
	}
}

esp_err_t eh_cp_feat_nw_split_get_status(uint8_t iface,
                                                    eh_cp_feat_nw_split_status_t *status)
{
    return nw_split_get_status(iface, status);
}

esp_err_t eh_cp_feat_nw_split_set_config(uint8_t iface,
                                                    const char *ip, const char *nm,
                                                    const char *gw,
                                                    const char *dns_ip, uint8_t dns_type)
{
    return nw_split_set_config(iface, ip, nm, gw, dns_ip, dns_type);
}

#endif
