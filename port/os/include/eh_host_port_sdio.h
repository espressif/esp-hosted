/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Wrapper interfaces for SDMMC to communicated with slave using SDIO */

#ifndef __PORT_ESP_HOSTED_HOST_SDIO_H_
#define __PORT_ESP_HOSTED_HOST_SDIO_H_

#include "esp_check.h"
#include "eh_common_header.h"

#include "eh_common_interface.h"   /* ESP_TRANSPORT_SDIO_MAX_BUF_SIZE = 1536 */

/* Mirrors upstream host/drivers/transport/transport_drv.h:52 — the host
 * SDIO transport's max DMA buffer size on the wire.  Sourced from the
 * canonical ESP_TRANSPORT_SDIO_MAX_BUF_SIZE in eh_common_interface.h so
 * the constant is single-sourced across host (this file) and slave. */
#ifndef MAX_SDIO_BUFFER_SIZE
#define MAX_SDIO_BUFFER_SIZE             ESP_TRANSPORT_SDIO_MAX_BUF_SIZE
#endif
#define MAX_TRANSPORT_BUFFER_SIZE        MAX_SDIO_BUFFER_SIZE
#define ESP_HOSTED_SDIO_UNRESPONSIVE_CODE 0x107

#ifndef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE (MAX_TRANSPORT_BUFFER_SIZE-EH_ESP_PAYLOAD_HEADER_OFFSET)
#endif

/* Hosted init function to init the SDIO host
 * returns a pointer to the sdio context */
void * eh_host_port_sdio_init(void);

/* Hosted SDIO deinit function
 * expects a pointer to the sdio context */
int eh_host_port_sdio_deinit(void *ctx);

/* Hosted SDIO to initialise the SDIO card */
int eh_host_port_sdio_card_init(void *ctx, bool show_config);

/* Hosted SDIO to deinitialise the SDIO card */
int eh_host_port_sdio_card_deinit(void *ctx);

/* Hosted SDIO functions to read / write to slave scratch registers
 * and to read / write block data
 * If lock_required is true, call will hold a mutex for the duration of the call */
int eh_host_port_sdio_read_reg(void *ctx, uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
int eh_host_port_sdio_write_reg(void *ctx, uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
int eh_host_port_sdio_read_block(void *ctx, uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
int eh_host_port_sdio_write_block(void *ctx, uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);

/* Hosted SDIO function that will block waiting for a SDIO interrupt from the slave
 * returns when there is an interrupt or timeout */
int eh_host_port_sdio_wait_slave_intr(void *ctx, uint32_t ticks_to_wait);

#endif
