// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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

#ifndef __TRANSPORT_LAYER_INTERFACE_H
#define __TRANSPORT_LAYER_INTERFACE_H
#include "esp_err.h"
#include "driver/sdio_slave.h"

typedef void *wlan_buf_handle_t;

typedef enum {
	SDIO = 0,
	SPI = 1,
} transport_layer;

typedef enum {
	DEINIT,
	INIT,
	ACTIVE,
	DEACTIVE,
} INTERFACE_STATE;

typedef struct {
	union {
		sdio_slave_buf_handle_t sdio_buf_handle;
		wlan_buf_handle_t	wlan_buf_handle;
		void *priv_buffer_handle;
	};
	uint8_t if_type;
	uint8_t if_num;
	uint8_t *payload;
	size_t payload_len;

	void (*free_buf_handle)(void *buf_handle);
} interface_buffer_handle_t;

typedef struct {
	/*
	union {
	} phy_context;
	*/
	INTERFACE_STATE state;
}interface_handle_t;

typedef struct {
	interface_handle_t * (*init)(uint8_t capabilities);
	int32_t (*write)(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
	interface_buffer_handle_t * (*read)(interface_handle_t *handle);
	esp_err_t (*reset)(interface_handle_t *handle);
	void (*deinit)(interface_handle_t *handle);
} if_ops_t;

typedef struct {
	transport_layer type;
	void *priv;
	if_ops_t *if_ops;
	int (*event_handler)(uint8_t bitmap);
} interface_context_t;

interface_context_t * interface_insert_driver(int (*callback)(uint8_t val));
int interface_remove_driver();
#endif
