/* SPDX-License-Identifier: Apache-2.0 */
/* HCI byte-pipe internals — transport-private (bus <-> eh_host_mcu_hci only). */

#pragma once

#include <stdint.h>

struct esp_payload_header;

/* TX (host->CP): stash the H4 type byte in the header, body follows;
 * returns the resulting payload length. */
uint16_t eh_host_mcu_hci_tx_pack(struct esp_payload_header *hdr,
                                 uint8_t *dst, const uint8_t *src, uint16_t len);

/* Remove and return the H4 type byte so the HCI body can be re-framed.
 * Used by SPI/SPI-HD/UART; SDIO handles itself inside
 */
uint8_t eh_host_mcu_hci_take_type(uint8_t *body, uint16_t *body_len);

/* RX (CP->host): bus hands each inbound frame to the bound handler. */
void eh_host_mcu_hci_rx_deliver(const uint8_t *frame, uint16_t len);
