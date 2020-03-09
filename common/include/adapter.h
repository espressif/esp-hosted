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

#ifndef __ESP_NETWORK_ADAPTER__H
#define __ESP_NETWORK_ADAPTER__H

struct esp_payload_header {
	uint8_t 		 if_type:4;
	uint8_t 		 if_num:4;
	uint8_t			 reserved1;
	uint16_t                 len;
	uint16_t                 offset;
	uint8_t                  reserved2[2];
} __attribute__((packed));

typedef enum {
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_SERIAL_IF,
} ESP_INTERFACE_TYPE;

typedef enum {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
} ESP_HOST_INTERRUPT;

#endif
