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

#ifndef __ESP_COMMAND_H
#define __ESP_COMMAND_H

#include "adapter.h"
#include "interface.h"

struct l2_ethhdr {
    uint8_t h_dest[MAC_ADDR_LEN];
    uint8_t h_source[MAC_ADDR_LEN];
    uint16_t h_proto;
} __attribute__((packed));

struct macfilter_list {
    uint8_t count;
    uint8_t mac_addr[MAX_MULTICAST_ADDR_COUNT][MAC_ADDR_LEN];
};
#define ETH_P_PAE 0x8E88 /* Port Access Entity (IEEE 802.1X) */
#define ETH_P_EAPOL ETH_P_PAE

int process_init_interface(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_deinit_interface(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_start_scan(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_get_mac(uint8_t if_type);
int process_set_mac(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_sta_connect(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_sta_disconnect(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_add_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_del_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_set_default_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_auth_request(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_assoc_request(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_set_ip(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_set_mcast_mac_list(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_tx_power(uint8_t if_type, uint8_t *payload, uint16_t payload_len, uint8_t cmd_code);
int process_reg_set(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
int process_reg_get(uint8_t if_type, uint8_t *payload, uint16_t payload_len);

esp_err_t initialise_wifi(void);

inline esp_err_t send_command_response(interface_buffer_handle_t *buf_handle)
{
	return send_to_host(PRIO_Q_HIGH, buf_handle);
}

inline esp_err_t send_command_event(interface_buffer_handle_t *buf_handle)
{
	return send_to_host(PRIO_Q_HIGH, buf_handle);
}

inline esp_err_t send_frame_to_host(interface_buffer_handle_t *buf_handle)
{
	return send_to_host(PRIO_Q_HIGH, buf_handle);
}
#endif
