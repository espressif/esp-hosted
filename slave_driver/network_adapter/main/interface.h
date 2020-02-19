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

typedef enum {
	SDIO = 0,
	SPI = 1,
} transport_layer;

enum HOST_INTERRUPTS {
	START_DATA_PATH = 0,
	STOP_DATA_PATH,
	RESET,
};

struct payload_header {
	uint8_t 		 pkt_type:2;
	uint8_t 		 if_type:3;
	uint8_t 		 if_num:3;
	uint8_t			 reserved1;
	uint16_t                 len;
	uint16_t                 offset;
	uint8_t                  reserved2[2];
} __attribute__((packed));

typedef struct {
	esp_err_t (*init)();
	int32_t (*write)(uint8_t if_type, uint8_t if_num, uint8_t* data, int32_t len);
	esp_err_t (*read)(void **handle, uint8_t **out_addr, size_t *out_len);
	esp_err_t (*reset)();
	void (*read_post_process)(void *handle);
	void (*deinit)();
} if_ops_t;

typedef struct {
	transport_layer type;
	void *priv;
	if_ops_t *if_ops;
	int (*callback_func)(uint8_t bitmap);
} interface_context_t;

interface_context_t * insert_driver(int (*callback)(uint8_t val));
int remove_driver();
#endif
