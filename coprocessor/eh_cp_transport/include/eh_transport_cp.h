// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD

#ifndef __TRANSPORT_LAYER_INTERFACE_H
#define __TRANSPORT_LAYER_INTERFACE_H
#include "esp_err.h"
#include "eh_log.h"
#include "eh_transport.h"
#include "eh_frame.h"

/* Transport->core event codes. */
typedef enum {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
	ESP_POWER_SAVE_ON,
	ESP_POWER_SAVE_OFF,
	ESP_MAX_HOST_INTERRUPT,
} ESP_HOST_INTERRUPT;

#ifdef CONFIG_EH_TRANSPORT_CP_SDIO

#if CONFIG_SOC_SDIO_SLAVE_SUPPORTED
	#include "driver/sdio_slave.h"
#else
	#error "SDIO is not supported for this chipset"
#endif

/* SDIO stores sdio_slave_buf_handle_t in priv_buffer_handle. */
static inline sdio_slave_buf_handle_t ibuf_sdio_get(const interface_buffer_handle_t *h)
{
	return (sdio_slave_buf_handle_t)h->priv_buffer_handle;
}
static inline void ibuf_sdio_set(interface_buffer_handle_t *h, sdio_slave_buf_handle_t bh)
{
	h->priv_buffer_handle = (void *)bh;
}

#endif /* CONFIG_EH_TRANSPORT_CP_SDIO */

#ifdef CONFIG_EH_TRANSPORT_CP_SPI_HD
	#include "driver/spi_slave_hd.h"

static inline spi_slave_hd_data_t *ibuf_spi_hd_get(const interface_buffer_handle_t *h)
{
	return (spi_slave_hd_data_t *)h->priv_buffer_handle;
}
static inline void ibuf_spi_hd_set(interface_buffer_handle_t *h, spi_slave_hd_data_t *t)
{
	h->priv_buffer_handle = (void *)t;
}

#endif /* CONFIG_EH_TRANSPORT_CP_SPI_HD */

typedef enum {
	LENGTH_1_BYTE  = 1,
	LENGTH_2_BYTE  = 2,
	LENGTH_3_BYTE  = 3,
	LENGTH_4_BYTE  = 4,
} byte_length;

typedef enum {
	SDIO   = 0,
	SPI    = 1,
	SPI_HD = 2,
	UART   = 3,
} transport_layer;

typedef enum {
	DEINIT,
	DEACTIVE,
	ACTIVE,
} INTERFACE_STATE;

typedef struct {
	INTERFACE_STATE state;
} interface_handle_t;

#if CONFIG_EH_TRANSPORT_CP_SPI
#define MAX_TRANSPORT_BUF_SIZE ESP_TRANSPORT_SPI_MAX_BUF_SIZE
#elif CONFIG_EH_TRANSPORT_CP_SDIO
#define MAX_TRANSPORT_BUF_SIZE ESP_TRANSPORT_SDIO_MAX_BUF_SIZE
#elif CONFIG_EH_TRANSPORT_CP_SPI_HD
#define MAX_TRANSPORT_BUF_SIZE ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE
#elif CONFIG_EH_TRANSPORT_CP_UART
#define MAX_TRANSPORT_BUF_SIZE ESP_TRANSPORT_UART_MAX_BUF_SIZE
#endif

#define BSSID_BYTES_SIZE       6

typedef struct {
	interface_handle_t * (*init)(void);
	int32_t (*write)(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
	int (*read)(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
	esp_err_t (*reset)(interface_handle_t *handle);
	void (*deinit)(interface_handle_t *handle);
} if_ops_t;

typedef struct {
	transport_layer type;
	void *priv;
	if_ops_t *if_ops;
	int (*event_handler)(uint8_t bitmap);
} interface_context_t;

typedef struct {
	uint8_t throttle_high_threshold;
	uint8_t throttle_low_threshold;
} slave_config_t;

typedef struct {
	uint8_t current_throttling;
} slave_state_t;

interface_context_t * interface_insert_driver(int (*callback)(uint8_t val));
int interface_remove_driver(void);
void generate_startup_event(uint8_t cap, uint32_t ext_cap, uint8_t raw_tp_cap,
                            const uint32_t feat_caps[8]);
int send_to_host_queue(interface_buffer_handle_t *buf_handle, uint8_t queue_type);
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_NW_SPLIT_READY
void send_dhcp_dns_info_to_host(uint8_t network_up, uint8_t send_wifi_connected);
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

extern slave_config_t slv_cfg_g;
extern slave_state_t  slv_state_g;

#endif /* __TRANSPORT_LAYER_INTERFACE_H */
