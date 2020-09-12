// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//#include "unistd.h"
#include "string.h"
#include "cmsis_os.h"
#include "trace.h"
#include "serial_if.h"
#include "transport_pserial.h"
#include "platform_wrapper.h"

#define MILLISEC_TO_SEC			1000
static osSemaphoreId readSemaphore;
static serial_handle_t * serial_if_g;

static void control_path_rx_indication(void);

struct esp_hosted_driver_handle_t {
    int handle; /* dummy variable */
};

int control_path_platform_init(void)
{
	osSemaphoreDef(READSEM);

	/* control path semaphore */
	readSemaphore = osSemaphoreCreate(osSemaphore(READSEM) , 1);
	assert(readSemaphore);

	/* grab the semaphore, so that task will be mandated to wait on semaphore */
	if (osSemaphoreWait(readSemaphore , portMAX_DELAY) != osOK) {
	    printf("could not obtain readSemaphore\n\r");
	    return STM_FAIL;
	}

	serial_if_g = serial_init(control_path_rx_indication);
	if (!serial_if_g) {
	    printf("Serial interface creation failed\n\r");
	    assert(serial_if_g);
	    return STM_FAIL;
	}
	if (STM_OK != serial_if_g->fops->open(serial_if_g)) {
		printf("Serial interface open failed\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

int control_path_platform_deinit(void)
{
	if (STM_OK != serial_if_g->fops->close(serial_if_g)) {
		printf("Serial interface close failed\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

static void control_path_rx_indication(void)
{
	/* heads up to control path for read */
	if(readSemaphore) {
		osSemaphoreRelease(readSemaphore);
	}
}

void* esp_hosted_malloc(size_t size)
{
	return malloc(size);
}

void* esp_hosted_calloc(size_t blk_no, size_t size)
{
	void* ptr = malloc(blk_no*size);
	if (!ptr) {
		return NULL;
	}

	memset(ptr, 0, blk_no*size);
	return ptr;
}

void esp_hosted_free(void* ptr)
{
	if(ptr) {
		free(ptr);
		ptr=NULL;
	}
}

struct esp_hosted_driver_handle_t* esp_hosted_driver_open(const char* transport)
{
	struct esp_hosted_driver_handle_t* esp_hosted_driver_handle = NULL;
	if (!transport) {
		printf("Invalid parameter in open \n\r");
		return NULL;
	}
	esp_hosted_driver_handle = (struct esp_hosted_driver_handle_t*) esp_hosted_calloc
		(1,sizeof(struct esp_hosted_driver_handle_t));
	if (!esp_hosted_driver_handle) {
		printf("Failed to allocate memory \n");
		return NULL;
	}
	return esp_hosted_driver_handle;
}

int esp_hosted_driver_write (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle,
	uint8_t* buf, int in_count, int* out_count)
{
	int ret = 0;
	if (!esp_hosted_driver_handle || !buf || !in_count || !out_count) {
		printf("Invalid parameters in write\n\r");
		return STM_FAIL;
	}

	if( (!serial_if_g) ||
		(!serial_if_g->fops) ||
		(!serial_if_g->fops->write)) {
		printf("serial interface not valid\n\r");
		return STM_FAIL;
	}

	ret = serial_if_g->fops->write(serial_if_g, buf, in_count);
	if (ret != STM_OK) {
		*out_count = 0;
		printf("Failed to write data\n\r");
		return STM_FAIL;
	}

	*out_count = in_count;
	return STM_OK;
}

uint8_t* esp_hosted_driver_read (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle,
	int read_len, uint8_t wait, uint32_t* buf_len)
{
	uint16_t rx_buf_len = 0;
	uint8_t* read_buf = NULL;
	uint8_t* buf = NULL;
	int len = 0, ret = 0;

	if (!esp_hosted_driver_handle || !read_len || !buf_len || !wait) {
		printf("Invalid parameters in read\n\r");
		return NULL;
	}
	if(! readSemaphore) {
		printf("Semaphore not initialized\n\r");
		return NULL;
	}
	if (osSemaphoreWait(readSemaphore , wait * MILLISEC_TO_SEC ) != osOK) {
		printf("Failed to read data \n\r");
		return NULL;
	}

	if( (!serial_if_g) ||
		(!serial_if_g->fops) ||
		(!serial_if_g->fops->read)) {
		printf("serial interface refusing to read\n\r");
		return NULL;
	}

	read_buf = serial_if_g->fops->read(serial_if_g, &rx_buf_len);
	if ((! read_buf) || (!rx_buf_len)) {
		printf("serial read failed\n\r");
		return NULL;
	}
	print_hex_dump(read_buf, rx_buf_len, "Serial read data");

/*
 * Read Operation happens in two steps because total read length is unkown
 * at first read.
 *      1) Read fixed length of RX data
 *      2) Read variable length of RX data
 *
 * 1) Read fixed length of RX data :
 * Read fixed length of received data in below format:
 * ----------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
 * ----------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 *  ---------------------------------------------------------------------------
 *      1         |       2         | Endpoint Length |     1     |     2     |
 *  ---------------------------------------------------------------------------
 *
 *  read_len = 1 + 2 + Endpoint length + 1 + 2
 */

	len = min(rx_buf_len,read_len);
	buf = (uint8_t* )esp_hosted_calloc(1, len);
	if (!buf) {
		printf("Failed to allocate memory \n\r");
		goto err;
	}

	memset(buf, 0, read_len);
	memcpy(buf, read_buf, len);

 /* parse_tlv function returns variable payload length of received data in buf_len */
	ret = parse_tlv(buf, buf_len);
	if (ret != STM_OK) {
		esp_hosted_free(buf);
		printf("Failed to parse RX data \n\r");
		goto err;
	}

	esp_hosted_free(buf);

	buf = NULL;
/*
 * 2) Read variable length of RX data:
 */
	buf = (uint8_t* )esp_hosted_calloc(1, *buf_len);
	if (!buf) {
		printf("Failed to allocate memory \n\r");
		goto err;
	}

	memcpy((buf), read_buf+read_len, *buf_len);

	esp_hosted_free(read_buf);
	read_buf = NULL;
	return buf;
err:
	if (read_buf) {
		esp_hosted_free(read_buf);
	}
	return NULL;
}

int esp_hosted_driver_close(struct esp_hosted_driver_handle_t* esp_hosted_driver_handle)
{
	if (!esp_hosted_driver_handle) {
		printf("Invalid parameter in close \n\r");
	}
	esp_hosted_free(esp_hosted_driver_handle);
	return STM_OK;
}
