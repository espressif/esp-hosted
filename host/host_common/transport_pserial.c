// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

/** Includes **/
#include "string.h"
#include "transport_pserial.h"
#include "platform_wrapper.h"

/** Constants/Macros **/
#define SUCCESS                     0
#define FAILURE                    -1

#define SIZE_OF_TYPE                1
#define SIZE_OF_LENGTH              2

#define PROTO_PSER_TLV_T_EPNAME     0x01
#define PROTO_PSER_TLV_T_DATA       0x02

#ifdef STM32F469xx
#define command_log(format, ...) printf(format "\r", ##__VA_ARGS__);
#else
#define command_log(...) printf(__VA_ARGS__);
#endif

/** Exported variables **/
const char* transport = "/dev/esps0";
static const char* ep_name = "control";

// IMP: free buffer after reading protobuf data
/*
 * The data written on serial driver i.e `/dev/esps0`
 * In TLV i.e. Type Length Value format, to transfer data between host and ESP32
 *  | type | length | value |
 * Types are 0x01 : for endpoint name
 *           0x02 : for data
 * length is respective value field's data length in 16 bits
 * value is actual data to be transferred
 */

uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length)
{
	uint16_t count = 0;
	buf[count] = PROTO_PSER_TLV_T_EPNAME;
	count++;
	uint16_t ep_length = strlen(ep_name);
	buf[count] = (ep_length & 0xFF);
	count++;
	buf[count] = ((ep_length >> 8) & 0xFF);
	count++;
	strncpy((char *)&buf[count], ep_name, ep_length);
	count = count + ep_length;
	buf[count]= PROTO_PSER_TLV_T_DATA;
	count++;
	buf[count] = (data_length & 0xFF);
	count++;
	buf[count] = ((data_length >> 8) & 0xFF);
	count++;
	memcpy(&buf[count], data, data_length);
	count = count + data_length;
	return count;
}

uint8_t parse_tlv(uint8_t* data, uint32_t* pro_len)
{
	uint64_t len = 0;
	uint16_t val_len = 0;
	if (data[len] == PROTO_PSER_TLV_T_EPNAME) {
		len++;
		val_len = data[len];
		len++;
		val_len = (data[len] << 8) + val_len;
		if (val_len == strlen(ep_name)) {
			if (strncmp((char* )&data[len],ep_name,strlen(ep_name))) {
				len = len + strlen(ep_name) + 1;
				if (data[len] == PROTO_PSER_TLV_T_DATA) {
					len++;
					val_len = data[len];
					len++;
					val_len = (data[len] << 8) + val_len;
					len++;
					*pro_len = val_len;
					return SUCCESS;
				} else {
					command_log("Data Type not matched, exp %d, recvd %d\n",
							PROTO_PSER_TLV_T_DATA, data[len]);
				}
			} else {
				command_log("Endpoint Name not matched, exp %s, recvd %s\n",
						ep_name, (char* )&data[len]);
			}
		} else {
			command_log("Endpoint length not matched, exp %lu, recvd %d\n",
					(long unsigned int)(strlen(ep_name)), val_len);
		}
	} else {
		command_log("Endpoint type not matched, exp %d, recvd %d\n",
				PROTO_PSER_TLV_T_EPNAME, data[len]);
	}
	return FAILURE;
}

uint8_t * transport_pserial_data_handler(uint8_t* data, uint16_t data_length,
		uint8_t wait, uint32_t* pro_len)
{
	int count = 0, ret = 0;
	uint16_t buf_len = 0, read_len = 0;
	uint8_t *write_buf = NULL, *read_buf = NULL;
	struct esp_hosted_driver_handle_t* esp_hosted_driver_handle = NULL;

/*
 * TLV (Type - Length - Value) structure is as follows:
 * --------------------------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length | Data Value  |
 * --------------------------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 * --------------------------------------------------------------------------------------------
 *       1        |        2        | Endpoint length |     1     |      2      | Data length |
 * --------------------------------------------------------------------------------------------
 */
	buf_len = SIZE_OF_TYPE + SIZE_OF_LENGTH + strlen(ep_name) +
		SIZE_OF_TYPE + SIZE_OF_LENGTH + data_length;

	read_len = buf_len - data_length;

	write_buf = (uint8_t* )esp_hosted_calloc(1, buf_len);
	if (!write_buf) {
		command_log("Failed to allocate memory \n");
		return NULL;
	}

	esp_hosted_driver_handle = esp_hosted_driver_open(transport);
	if (!esp_hosted_driver_handle) {
		esp_hosted_free(write_buf);
		write_buf = NULL;
		return NULL;
	}

	count = compose_tlv(write_buf, data, data_length);
	if (!count) {
		command_log("Failed to compose TX data \n");
		goto err;
	}

	ret = esp_hosted_driver_write(esp_hosted_driver_handle, write_buf, count, &count);
	if (ret != SUCCESS) {
		command_log("write error \n");
		goto err;
	}

	read_buf = esp_hosted_driver_read(esp_hosted_driver_handle, read_len, wait, pro_len);
	if (! read_buf) {
		command_log("Failed to read RX data \n");
		goto err;
	}

	ret = esp_hosted_driver_close(&esp_hosted_driver_handle);
	if (ret != SUCCESS) {
		command_log("Failed to close driver interface \n");
	}

	return read_buf;
err:
	if (write_buf) {
		esp_hosted_free(write_buf);
		write_buf = NULL;
	}

	ret = esp_hosted_driver_close(&esp_hosted_driver_handle);
	if (ret != SUCCESS) {
		command_log("Failed to close driver interface \n");
	}
	return NULL;
}
