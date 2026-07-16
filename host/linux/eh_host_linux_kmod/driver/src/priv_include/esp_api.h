// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef _esp_api__h_
#define _esp_api__h_

#include "esp.h"

int esp_add_card(struct esp_adapter *adapter);
int esp_remove_card(struct esp_adapter *adapter);
void esp_process_new_packet_intr(struct esp_adapter *adapter);
struct esp_adapter * esp_get_adapter(void);
int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb);
u8 esp_is_bt_supported_over_sdio(u32 cap);
int esp_is_tx_queue_paused(void);
void esp_tx_pause(void);
void esp_tx_resume(void);
int process_init_event(u8 *evt_buf, u8 len);
int send_host_caps_reply(struct esp_adapter *adapter);
void process_capabilities(u8 cap);
void process_test_capabilities(u8 cap);
int is_host_sleeping(void);
extern volatile u8 stop_data;

/* Diagnose the EH_PRIV_RPC_VERSION (0x1A) value parsed from the CP init
 * event.  Logs a warning + concrete recovery instructions on mismatch
 * or absence, but never refuses to attach: the host must stay up so the
 * OTA channel remains available — re-flashing the CP is the only
 * recovery, and the host has to be reachable for that to happen.
 *
 * @peer_rpc_version: value parsed from EH_PRIV_RPC_VERSION TLV (0 if
 *                    the TLV was not present in the CP init event).
 * Returns true if the peer advertised the TLV (any non-zero value).
 * Caller stores this on adapter->peer_advertised_rpc_version so
 * send_host_caps_reply() knows whether to echo 0x1A back; older CP
 * parsers may not skip unknown tags silently.
 */
bool diagnose_peer_rpc_version(u8 peer_rpc_version);

#endif
