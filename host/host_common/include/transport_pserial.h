// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

/** prevent recursive inclusion **/
#ifndef __TRANSPORT_PSERIAL_H
#define __TRANSPORT_PSERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length);

uint8_t parse_tlv(uint8_t* data, uint32_t* pro_len);

uint8_t * transport_pserial_data_handler(uint8_t *data, uint16_t data_length, uint8_t wait, uint32_t* pro_len);
#endif
