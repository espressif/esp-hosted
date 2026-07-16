/* SPDX-License-Identifier: Apache-2.0 */
/* HCI byte-pipe wire glue (MCU transport, bus-agnostic).
 *
 * host→CP carries the H4 type byte in the wire header's pkt_type slot
 * (body kept pure); the CP rebuilds the contiguous H4 frame for its
 * VHCI. CP→host carries the H4 byte inline at payload[0], delivered
 * straight to the registered handler. Lives here (always compiled) so
 * every bus shares one implementation. */

#include <stdint.h>
#include <string.h>

#include "eh_common_header.h"               /* struct esp_payload_header */
#include "eh_host_mcu_transport_channels.h" /* rx_handler typedef + set_rx_handler */
#include "eh_host_mcu_hci_internal.h"       /* tx_pack + rx_deliver */

/* TX (host→CP): stash the H4 type byte in the header, body follows. */
uint16_t eh_host_mcu_hci_tx_pack(struct esp_payload_header *hdr,
		uint8_t *dst, const uint8_t *src, uint16_t len)
{
	hdr->hci_pkt_type = src[0];
	len -= 1;
	memcpy(dst, &src[1], len);
	return len;
}

/* RX (CP→host): deliver to the bound handler (defaults to drop). */
static void rx_drop(const uint8_t *frame, uint16_t len, void *user)
{ (void)frame; (void)len; (void)user; }
static eh_host_mcu_hci_rx_handler_fn s_rx_handler = rx_drop;
static void                         *s_rx_user;

void eh_host_mcu_hci_set_rx_handler(eh_host_mcu_hci_rx_handler_fn fn, void *user)
{
	s_rx_handler = fn ? fn : rx_drop;
	s_rx_user    = fn ? user : NULL;
}

void eh_host_mcu_hci_rx_deliver(const uint8_t *frame, uint16_t len)
{
	s_rx_handler(frame, len, s_rx_user);
}
