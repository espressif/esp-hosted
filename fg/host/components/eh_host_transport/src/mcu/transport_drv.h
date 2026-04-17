/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** prevent recursive inclusion **/
#ifndef __TRANSPORT_DRV_H
#define __TRANSPORT_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/

#include "esp_err.h"

#include "eh_transport.h"
#include "eh_api_types.h"
#include "eh_interface.h"
#include "eh_header.h"

#include "port_eh_host_config.h"

#if H_TRANSPORT_IN_USE == H_TRANSPORT_SPI
#include "port_eh_host_spi.h"
#elif H_TRANSPORT_IN_USE == H_TRANSPORT_SDIO
#include "port_eh_host_sdio.h"
#elif H_TRANSPORT_IN_USE == H_TRANSPORT_SPI_HD
#include "port_eh_host_spi_hd.h"
#elif H_TRANSPORT_IN_USE == H_TRANSPORT_UART
#include "port_eh_host_uart.h"
#endif

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      (0xD)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C5      (0x17)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C61     (0x14)

#define MAX_SPI_BUFFER_SIZE               ESP_TRANSPORT_SPI_MAX_BUF_SIZE
#define MAX_SDIO_BUFFER_SIZE              ESP_TRANSPORT_SDIO_MAX_BUF_SIZE
#define MAX_SPI_HD_BUFFER_SIZE            ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE
#define MAX_UART_BUFFER_SIZE              ESP_TRANSPORT_UART_MAX_BUF_SIZE

#ifndef BIT
#define BIT(x)                            (1UL << (x))
#endif

#ifndef BIT64
#define BIT64(nr)               (1ULL << (nr))
#endif

#define H_MIN(a, b) (((a) < (b)) ? (a) : (b))

#define MHZ_TO_HZ(x) (1000000*(x))

#define H_FREE_PTR_WITH_FUNC(FreeFunc, FreePtr) do {	\
	if (FreeFunc && FreePtr) {             \
		FreeFunc(FreePtr);                 \
		FreePtr = NULL;                    \
	}                                      \
} while (0);

#define SUCCESS 0
#define FAILURE -1

typedef enum {
	TRANSPORT_INACTIVE,
	TRANSPORT_RX_ACTIVE,
	TRANSPORT_TX_ACTIVE,
} transport_drv_events_e;

/* interface header */
typedef struct {
	union {
		void *priv_buffer_handle;
	};
	uint8_t if_type;
	uint8_t if_num;
	uint8_t *payload;
	uint8_t flag;
	uint16_t payload_len;
	uint16_t seq_num;
	/* no need of memcpy at different layers */
	uint8_t payload_zcopy;

	void (*free_buf_handle)(void *buf_handle);
} interface_buffer_handle_t;

struct esp_private {
	uint8_t     if_type;
	uint8_t     if_num;
	void        *netdev;
};

struct hosted_transport_context_t {
	uint8_t  *tx_buf;
	uint32_t  tx_buf_size;
	uint8_t  *rx_buf;
};

extern volatile uint8_t wifi_tx_throttling;
extern volatile uint8_t host_hdr_ver_agreed; /* agreed wire-header version (1=V1, 2=V2) */
extern volatile uint8_t host_rpc_ver_agreed; /* agreed RPC version (1=V1 variant, 2=V2 unified) */

typedef int (*hosted_rxcb_t)(void *buffer, uint16_t len, void *free_buff_hdl);

typedef void (transport_free_cb_t)(void* buffer);
typedef esp_err_t (*transport_channel_tx_fn_t)(void *h, void *buffer, size_t len);
typedef esp_err_t (*transport_channel_rx_fn_t)(void *h, void *buffer, void * buff_to_free, size_t len);

typedef struct {
	void * api_chan;
	eh_if_type_t if_type;
	uint8_t secure;
	transport_channel_tx_fn_t tx;
	transport_channel_rx_fn_t rx;
	void *memp;
} transport_channel_t;


esp_err_t setup_transport(void(*eh_host_up_cb)(void));
esp_err_t teardown_transport(void);
esp_err_t transport_drv_reconfigure(void);
transport_channel_t *transport_drv_add_channel(void *api_chan,
		eh_if_type_t if_type, uint8_t secure,
		transport_channel_tx_fn_t *tx, const transport_channel_rx_fn_t rx);
esp_err_t transport_drv_remove_channel(transport_channel_t *channel);


void *bus_init_internal(void);
void bus_deinit_internal(void *bus_handle);

void process_priv_communication(interface_buffer_handle_t *buf_handle);

esp_err_t send_slave_config(uint8_t host_cap, uint8_t firmware_chip_id,
		uint8_t raw_tp_direction, uint8_t low_thr_thesh, uint8_t high_thr_thesh);

uint8_t is_transport_rx_ready(void);
uint8_t is_transport_tx_ready(void);

#define H_BUFF_NO_ZEROCOPY 0
#define H_BUFF_ZEROCOPY 1

#define H_DEFLT_FREE_FUNC g_h.funcs->_h_free

#define MAX_RETRY_TRANSPORT_ACTIVE 100


int eh_host_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t *payload_buf, uint16_t payload_len, uint8_t buff_zerocopy,
		uint8_t *buffer_to_free, void (*free_buf_func)(void *ptr), uint8_t flags);

int serial_rx_handler(interface_buffer_handle_t * buf_handle);
void set_transport_state(uint8_t state);

int ensure_slave_bus_ready(void *bus_handle);
void check_if_max_freq_used(uint8_t chip_type);

int bus_inform_slave_host_power_save_start(void);
int bus_inform_slave_host_power_save_stop(void);

#ifdef __cplusplus
}
#endif

#endif
