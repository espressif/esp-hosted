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

typedef enum {
	SDIO = 0,
	SPI = 1,
} transport_layer;

enum HOST_INTERRUPTS {
	START_DATA_PATH = 0,
	STOP_DATA_PATH,
	RESET,
};

typedef struct {
	union {
		sdio_slave_buf_handle_t buf_handle;
	};
}interface_handle_t;

typedef struct {
	esp_err_t (*init)();
	int32_t (*write)(uint8_t if_type, uint8_t if_num, uint8_t* payload, int32_t payload_len);
	esp_err_t (*read)(interface_handle_t *handle, uint8_t **out_addr, size_t *out_len);
	esp_err_t (*reset)();
	void (*read_post_process)(interface_handle_t *handle);
	void (*deinit)();
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
