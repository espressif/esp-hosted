// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

/** prevent recursive inclusion **/
#ifndef __SERIAL_IF_H
#define __SERIAL_IF_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "adapter.h"

#define SIZE_OF_TYPE                1
#define SIZE_OF_LENGTH              2

/*
 * The data written on serial driver file, `SERIAL_IF_FILE` from adapter.h
 * In TLV i.e. Type Length Value format, to transfer data between host and ESP32
 *  | type | length | value |
 * Types are 0x01 : for endpoint name
 *           0x02 : for data
 * length is respective value field's data length in 16 bits
 * value is actual data to be transferred
 */
uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length);

/* Parse the protobuf encoded data in format of tag, length and value
 * Thi will help application to decode protobuf payload and payload length
 **/
uint8_t parse_tlv(uint8_t* data, uint32_t* pro_len);

/* Open the serial driver for serial operations
 **/
int transport_pserial_open(void);

/* Close the serial driver for serial operations
 **/
int transport_pserial_close(void);

/* Send buffer with length as argument on transport as serial interface type
 **/
int transport_pserial_send(uint8_t* data, uint16_t data_length);

/* Read and return number of bytes and buffer from serial interface
 **/
uint8_t * transport_pserial_read(uint32_t *out_nbyte);
/*
 * control_path_platform_init function initializes the control
 * path data structures
 * Input parameter
 *      None
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_init(void);

/*
 * control_path_platform_deinit function cleans up the control
 * path library data structure
 * Input parameter
 *      None
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_deinit(void);
#endif
