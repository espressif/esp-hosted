// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

/** Include **/
#include "string.h"
#include "trace.h"
#include "util.h"

#include "app_main_api.h"
#include "netdev_api.h"
#include "arp_server_stub.h"


/** constants, macros **/
#define DEBUG_STREAM_ENABLED             1

#define PROTOCOL_LEN_ARP                 2
#define PROTOCOL_ARP                     0x0806
#define ARPING_OFFSET_PROTOCOL           12

#define OPCODE_LEN                       2

#define ARPING_OFFSET_SRC_MAC            6
#define ARPING_OFFSET_SRC_REPEAT_MAC     22
#define ARPING_OFFSET_DST_MAC            0
#define ARPING_OFFSET_DST_REPEAT_MAC     32
#define ARPING_OFFSET_SRC_IP             28
#define ARPING_OFFSET_DST_IP             38
#define ARPING_OFFSET_OPCODE             20
#define ARPING_MAX_PKT_SIZE              42

enum {
	ARP_REQ = 1,
	ARP_REPLY,
	MAX_MSG_TYPE_ARPING
};

/** Exported variables **/

static uint8_t arp_req_tx[ARPING_MAX_PKT_SIZE] =
{
/* arping */
	// dst mac (0th byte)   0xa0, 0x88, 0xb4, 0xe5, 0xd5, 0x38 //laptop
	// src mac (6th byte)   0x3c, 0x71, 0xbf, 0x9a, 0xbc, 0xb8 //stm
	// src mac (22th byte)  0x3c, 0x71, 0xbf, 0x9a, 0xbc, 0xb8 //stm
	// src ip  (28th byte)  192.168.1.233                      //stm
	// dst ip  (38th byte)  192.168.1.206                      //laptop
/*0000*/   0xa0, 0x88, 0xb4, 0xe5, 0xd5, 0x38, 0x3c, 0x71, 0xbf, 0x9a, 0xbc, 0xb8, 0x08, 0x06, 0x00, 0x01,
/*0010*/   0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0x3c, 0x71, 0xbf, 0x9a, 0xbc, 0xb8,  192,  168,    1,  233,
/*0020*/   0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  192,  168,    1,  206,

};


/** Function Definitions **/

/** Local functions **/

/**
  * @brief  Set ip address for arping
  * @param  stream - output buffer
  *         type - src/dest type of ip addr
  *         ip   - ip addr to set
  * @retval None
  */
static void arping_set_ipaddr(uint8_t *stream, ie_type_e type, uint32_t ip)
{
	uint32_t ip_nw = hton_long(ip);
	if (type == IP_ADDR_TYPE_SRC ) {
		stream_set(stream, &ip_nw, ARPING_OFFSET_SRC_IP, IP_ADDR_LEN);
	} else if (type == IP_ADDR_TYPE_DST ) {
		stream_set(stream, &ip_nw, ARPING_OFFSET_DST_IP, IP_ADDR_LEN);
	} else {
		printf("Address type be either src/dst\n");
	}
}

/**
  * @brief  Get ip address for arping
  * @param  stream - input buffer
  *         type - src/dest type of ip addr
  * @retval ip addr in 32 bits
  */
static uint32_t arping_get_ipaddr(uint8_t *stream, ie_type_e type)
{
	uint32_t ip_nw;
	if (type == IP_ADDR_TYPE_SRC ) {
		ip_nw = *(uint32_t*) stream_get(stream,
				ARPING_OFFSET_SRC_IP, IP_ADDR_LEN);
	} else if (type == IP_ADDR_TYPE_DST ) {
		ip_nw = *(uint32_t*) stream_get(stream,
				ARPING_OFFSET_DST_IP, IP_ADDR_LEN);
	} else {
		printf("Address type be either src/dst\n");
	}
	return ntoh_long(ip_nw);
}

/**
  * @brief  Set mac address
  * @param  stream - output buffer
  *         type - src/dest type of ip addr
  *         mac - input mac addr
  * @retval None
  */
static void arping_set_mac(uint8_t *stream, ie_type_e type, const uint8_t *mac)
{
	if (type == MAC_ADDR_TYPE_SRC) {
		stream_set(stream, mac, ARPING_OFFSET_SRC_MAC, MAC_LEN);
	} else if (type == MAC_ADDR_TYPE_DST) {
		stream_set(stream, mac, ARPING_OFFSET_DST_MAC, MAC_LEN);
	} else if (type == MAC_ADDR_TYPE_SRC_REPEAT) {
		stream_set(stream, mac, ARPING_OFFSET_SRC_REPEAT_MAC, MAC_LEN);
	} else if (type == MAC_ADDR_TYPE_DST_REPEAT) {
		stream_set(stream, mac, ARPING_OFFSET_DST_REPEAT_MAC, MAC_LEN);
	} else {
		printf("mac type be either src/dst/repeatSrc\n");
	}
}

/**
  * @brief  Get mac address
  * @param  stream - input buffer
  *         type - src/dest type of ip addr
  * @retval output mac addr ptr
  */
static uint8_t * arping_get_mac(uint8_t *stream, ie_type_e type)
{
	uint8_t * mac= 0;

	if (type == MAC_ADDR_TYPE_SRC) {
		mac = stream_get(stream, ARPING_OFFSET_SRC_MAC, MAC_LEN);
	} else if (type == MAC_ADDR_TYPE_DST) {
		mac = stream_get(stream, ARPING_OFFSET_DST_MAC, MAC_LEN);
	} else if (type == MAC_ADDR_TYPE_SRC_REPEAT) {
		mac = stream_get(stream, ARPING_OFFSET_SRC_REPEAT_MAC, MAC_LEN);
	} else if (type == MAC_ADDR_TYPE_DST_REPEAT) {
		mac = stream_get(stream, ARPING_OFFSET_DST_REPEAT_MAC, MAC_LEN);
	} else {
		printf("mac type be either src/dst/srcRepeat\n");
	}
	return mac;
}

/**
  * @brief  Set opcode
  * @param  stream - input buffer
  *         opcode - request/reply type
  * @retval None
  */
static void arping_set_opcode(uint8_t *stream, const uint16_t opcode)
{
	uint16_t opcode_nw = hton_short(opcode);
	stream_set(stream, &opcode_nw, ARPING_OFFSET_OPCODE, OPCODE_LEN);
}

/**
  * @brief  Get opcode
  * @param  stream - input buffer
  * @retval opcode - request/reply type
  */
static uint16_t arping_get_opcode(uint8_t *stream)
{
	uint16_t opcode_nw = 0;
	opcode_nw = *(uint16_t*)stream_get(stream,
			ARPING_OFFSET_OPCODE, OPCODE_LEN);
	return (ntoh_short(opcode_nw));
}


/**
  * @brief  Get protocol
  * @param  stream - input buffer
  * @retval protocol field
  */
static uint16_t arping_get_protocol(uint8_t *stream)
{
	uint16_t prot_nw = 0;
	prot_nw = *(uint16_t*)stream_get(stream,
			ARPING_OFFSET_PROTOCOL, PROTOCOL_LEN_ARP);
	return (ntoh_short(prot_nw));
}


/**
  * @brief  Change ip addresses of src and dest
  * @param  stream - input/output buffer
  * @retval none
  */
static void arping_swap_ip_addresses(uint8_t * stream)
{
	uint32_t in_src_ip = 0;
	uint32_t in_dst_ip = 0;
	if (! stream)
	{
		printf("stream NULL passed\n\r");
		return;
	}

	in_src_ip = arping_get_ipaddr(stream, IP_ADDR_TYPE_SRC);
	in_dst_ip = arping_get_ipaddr(stream, IP_ADDR_TYPE_DST);

	arping_set_ipaddr(stream, IP_ADDR_TYPE_DST, in_src_ip);
	arping_set_ipaddr(stream, IP_ADDR_TYPE_SRC, in_dst_ip);
}

/**
  * @brief  change mac addr for src/dest
  * @param  stream - input/output buffer
  *         self_mac - source mac
  * @retval none
  */
static void arping_change_mac_addresses(uint8_t * stream,
		const uint8_t *self_mac)
{
	uint8_t * rsp_mac = NULL;

	if (! stream)
	{
		printf("stream NULL passed\n\r");
		return;
	}

	rsp_mac = arping_get_mac(stream, MAC_ADDR_TYPE_SRC);

	arping_set_mac(stream, MAC_ADDR_TYPE_DST, rsp_mac);
	arping_set_mac(stream, MAC_ADDR_TYPE_DST_REPEAT, rsp_mac);
	arping_set_mac(stream, MAC_ADDR_TYPE_SRC, self_mac);
	arping_set_mac(stream, MAC_ADDR_TYPE_SRC_REPEAT, self_mac);
}

/**
  * @brief  check if arp type of protocol
  * @param  pkt - input buffer
  * @retval true/false
  */
static uint8_t is_arp_packet(uint8_t *pkt)
{
	uint16_t prot = arping_get_protocol(pkt);
	uint16_t opcode = arping_get_opcode(pkt);
	if (PROTOCOL_ARP == prot) {
		if (opcode < MAX_MSG_TYPE_ARPING) {
			return opcode;
		}
	}
	return 0;
}

/**
  * @brief  Display arp pkt content
  * @param  pkt - input buffer
  *         pkt_len - len of pkt
  *         arp_msg - opcode of pkt
  * @retval None
  */
static void display_arp(uint8_t *pkt, uint16_t pkt_len, uint8_t arp_msg)
{
	char ip_addr_s[16];
	char mac_s[30];
	uint8_t *mac = NULL;
	char str_req[2][20] = {"ARP_REQ_RCVD", "ARP_RSP_RCVD"};
	static uint32_t count = 0;

	if (arp_msg < MAX_MSG_TYPE_ARPING) {
		ipv4_addr_ntoa(arping_get_ipaddr(pkt, IP_ADDR_TYPE_SRC), ip_addr_s, 20);
		mac = arping_get_mac(pkt, MAC_ADDR_TYPE_SRC);

		snprintf(mac_s, 30, "%02x:%02x:%02x:%02x:%02x:%02x",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

		printf("0x%05lX %s: %u bytes from %s (%s)\n\r",
				++count, str_req[arp_msg-1], pkt_len,  ip_addr_s, mac_s);
	}
}

/**
  * @brief  check if pkt destination is me
  * @param  src_ip - self ip
  *         pkt - input buffer
  * @retval 1 if pass, else 0
  */
static uint8_t is_arp_pkt_for_me(uint32_t *src_ip, uint8_t *pkt)
{
	uint32_t in_dst_ip = 0;

	in_dst_ip = arping_get_ipaddr(pkt, IP_ADDR_TYPE_DST);
	if (! is_same_buff(&in_dst_ip, src_ip, IP_ADDR_LEN) ){
		return 0;
	}

	return 1;
}


/** Exported Functions **/

/**
  * @brief  check and handle input stream for arp packet
  * @param  src_ip - self ip
  *         src_mac - self mac addr
  *         pkt - input buffer
  *         pkt_len - pkt size
  *         resp_len - rsp pkt size
  * @retval arp response buffer if valid arp request or NULL
  */
uint8_t * arp_req_handler(uint32_t *src_ip, const uint8_t *src_mac,
		uint8_t *pkt, uint16_t pkt_len, uint16_t *resp_len)
{
	uint8_t is_arp_req = 0;
	uint8_t *arp_resp = NULL;

	if (! pkt_len || ! pkt) {
		if(resp_len)
			*resp_len = 0;
		return NULL;
	}

	/* discard if not arp pkt */
	is_arp_req = is_arp_packet(pkt);
	if(! is_arp_req) {
		return NULL;
	}

	if (! is_arp_pkt_for_me(src_ip, pkt)) {
		return NULL;
	}

	display_arp(pkt, pkt_len, is_arp_req);

	if(ARP_REPLY == is_arp_req) {
		return NULL;
	}

	arp_resp = malloc(ARPING_MAX_PKT_SIZE);
	assert(arp_resp);


	/* replicate packet */
	memset(arp_resp, 0, ARPING_MAX_PKT_SIZE);
	memcpy(arp_resp, pkt, ARPING_MAX_PKT_SIZE);

	/* set opcode */
	arping_set_opcode(arp_resp, 2);

	/* swap source and destination ip addresses */
	arping_swap_ip_addresses(arp_resp);
	arping_change_mac_addresses(arp_resp, src_mac);

	if(resp_len)
		*resp_len = pkt_len;

	return arp_resp;
}

/**
  * @brief  send arping request
  * @param  net_handle - network handle
  *         src_mac - source mac addr
  *         src_ip - source ip addr
  *         dst_mac - destination mac addr
  *         dst_ip - destination ip addr
  * @retval response if valid arp request or NULL
  */
stm_ret_t send_arp_req(struct network_handle *net_handle, uint8_t *src_mac,
		uint32_t *src_ip, uint8_t *dst_mac, uint32_t *dst_ip)
{
	struct pbuf *buffer = NULL;
	int ret;

	if (!net_handle || !src_mac || !src_ip || !dst_mac || !dst_ip)
		return STM_FAIL;

	arping_set_mac(arp_req_tx, MAC_ADDR_TYPE_SRC, src_mac);
	arping_set_mac(arp_req_tx, MAC_ADDR_TYPE_SRC_REPEAT, src_mac);

	/* modify the request template */
	arping_set_ipaddr(arp_req_tx, IP_ADDR_TYPE_DST, *dst_ip);
	arping_set_ipaddr(arp_req_tx, IP_ADDR_TYPE_SRC, *src_ip);
	arping_set_mac(arp_req_tx, MAC_ADDR_TYPE_DST, dst_mac);
	arping_set_mac(arp_req_tx, MAC_ADDR_TYPE_DST_REPEAT, dst_mac);

	buffer = malloc(sizeof(struct pbuf));
	assert(buffer);

	buffer->payload = malloc(ARPING_MAX_PKT_SIZE);
	assert(buffer->payload);

	buffer->len = 42;

	memcpy(buffer->payload, arp_req_tx, ARPING_MAX_PKT_SIZE);

	ret = network_write(net_handle, buffer);

	return ret;
}

