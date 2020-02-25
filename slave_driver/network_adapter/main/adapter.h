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

#ifndef __NETWORK_ADAPTER__H
#define __NETWORK_ADAPTER__H

enum PACKET_TYPE {
	DATA_PACKET = 0,
};

enum INTERFACE_TYPE {
	STA_INTF = 0,
	AP_INTF,
	SERIAL_INTF = (1<<1),
};

typedef struct {
	uint8_t		if_type;
	uint8_t		if_num;
	uint16_t	len;
	void 		*buf;
	void		*buf_ptr;
} buf_descriptor_t;

typedef struct {
	interface_context_t *context;
} adapter;
#endif
