// Copyright 2015-2023 Espressif Systems (Shanghai) PTE LTD
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

#include "serial_if.h"
#include "serial_ll_if.h"
#include "trace.h"

struct serial_drv_handle_t {
	int handle; /* dummy variable */
};

static serial_ll_handle_t * serial_ll_if_g;
static void * readSemaphore;

#define HOSTED_CALLOC(buff,nbytes) do {                           \
    buff = (uint8_t *)g_h.funcs->_h_calloc(1, nbytes);                   \
    if (!buff) {                                                  \
        printf("%s, Failed to allocate memory \n", __func__);     \
        goto free_bufs;                                           \
    }                                                             \
} while(0);


static void control_path_rx_indication(void);

/* -------- Serial Drv ---------- */
struct serial_drv_handle_t* serial_drv_open(const char *transport)
{
	struct serial_drv_handle_t* serial_drv_handle = NULL;
	if (!transport) {
		printf("Invalid parameter in open \n\r");
		return NULL;
	}

	if(serial_drv_handle) {
		printf("return orig hndl\n");
		return serial_drv_handle;
	}

	serial_drv_handle = (struct serial_drv_handle_t*) g_h.funcs->_h_calloc
		(1,sizeof(struct serial_drv_handle_t));
	if (!serial_drv_handle) {
		printf("Failed to allocate memory \n");
		return NULL;
	}

	return serial_drv_handle;
}

int serial_drv_write (struct serial_drv_handle_t* serial_drv_handle,
	uint8_t* buf, int in_count, int* out_count)
{
	int ret = 0;
	if (!serial_drv_handle || !buf || !in_count || !out_count) {
		printf("Invalid parameters in write\n\r");
		return RET_INVALID;
	}

	if( (!serial_ll_if_g) ||
		(!serial_ll_if_g->fops) ||
		(!serial_ll_if_g->fops->write)) {
		printf("serial interface not valid\n\r");
		return RET_INVALID;
	}

	print_hex_dump(buf, in_count,"serial_tx");
	ret = serial_ll_if_g->fops->write(serial_ll_if_g, buf, in_count);
	if (ret != RET_OK) {
		*out_count = 0;
		printf("Failed to write data\n\r");
		return RET_FAIL;
	}

	*out_count = in_count;
	return RET_OK;
}


uint8_t * serial_drv_read(struct serial_drv_handle_t *serial_drv_handle,
		uint32_t *out_nbyte)
{
	uint16_t init_read_len = 0;
	uint16_t rx_buf_len = 0;
	uint8_t* read_buf = NULL;
	int ret = 0;
	/* Any of `CTRL_EP_NAME_EVENT` and `CTRL_EP_NAME_RESP` could be used,
	 * as both have same strlen in adapter.h */
	const char* ep_name = CTRL_EP_NAME_RESP;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;


	if (!serial_drv_handle || !out_nbyte) {
		printf("Invalid parameters in read\n\r");
		return NULL;
	}

	*out_nbyte = 0;

	if(!readSemaphore) {
		printf("Semaphore not initialized\n\r");
		return NULL;
	}

	//if (osSemaphoreWait(readSemaphore, HOSTED_BLOCK_MAX) != osOK) {
	//	printf("Failed to read data \n\r");
	//	return NULL;
	//}

	g_h.funcs->_h_get_semaphore(readSemaphore, HOSTED_BLOCK_MAX);

	if( (!serial_ll_if_g) ||
		(!serial_ll_if_g->fops) ||
		(!serial_ll_if_g->fops->read)) {
		printf("serial interface refusing to read\n\r");
		return NULL;
	}

	/* Get buffer from serial interface */
	read_buf = serial_ll_if_g->fops->read(serial_ll_if_g, &rx_buf_len);
	if ((!read_buf) || (!rx_buf_len)) {
		printf("serial read failed\n\r");
		return NULL;
	}
	print_hex_dump(read_buf, rx_buf_len, "Serial read data");

/*
 * Read Operation happens in two steps because total read length is unknown
 * at first read.
 *      1) Read fixed length of RX data
 *      2) Read variable length of RX data
 *
 * (1) Read fixed length of RX data :
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
 *  int_read_len = 1 + 2 + Endpoint length + 1 + 2
 */

	init_read_len = SIZE_OF_TYPE + SIZE_OF_LENGTH + strlen(ep_name) +
		SIZE_OF_TYPE + SIZE_OF_LENGTH;

	if(rx_buf_len < init_read_len) {
		mem_free(read_buf);
		printf("Incomplete serial buff, return\n");
		return NULL;
	}

	HOSTED_CALLOC(buf,init_read_len);

	g_h.funcs->_h_memcpy(buf, read_buf, init_read_len);

	/* parse_tlv function returns variable payload length
	 * of received data in buf_len
	 **/
	ret = parse_tlv(buf, &buf_len);
	if (ret || !buf_len) {
		mem_free(buf);
		printf("Failed to parse RX data \n\r");
		goto free_bufs;
	}

	if (rx_buf_len < (init_read_len + buf_len)) {
		printf("Buf read on serial iface is smaller than expected len\n");
		mem_free(buf);
		goto free_bufs;
	}

	mem_free(buf);
/*
 * (2) Read variable length of RX data:
 */
	HOSTED_CALLOC(buf,buf_len);

	g_h.funcs->_h_memcpy((buf), read_buf+init_read_len, buf_len);

	mem_free(read_buf);

	*out_nbyte = buf_len;
	return buf;

free_bufs:
	mem_free(read_buf);
	mem_free(buf);
	return NULL;
}

int serial_drv_close(struct serial_drv_handle_t** serial_drv_handle)
{
	if (!serial_drv_handle || !(*serial_drv_handle)) {
		printf("Invalid parameter in close \n\r");
		if (serial_drv_handle)
			mem_free(serial_drv_handle);
		return RET_INVALID;
	}
	mem_free(*serial_drv_handle);
	return RET_OK;
}

int control_path_platform_init(void)
{
	//osSemaphoreDef(READSEM);

	/* control path semaphore */
	//readSemaphore = osSemaphoreCreate(osSemaphore(READSEM) , 1);
	readSemaphore = g_h.funcs->_h_create_binary_semaphore();
	assert(readSemaphore);

	/* grab the semaphore, so that task will be mandated to wait on semaphore */
	//if (osSemaphoreWait(readSemaphore , HOSTED_BLOCK_MAX) != osOK) {
	g_h.funcs->_h_get_semaphore(readSemaphore, HOSTED_BLOCK_MAX);

	serial_ll_if_g = serial_ll_init(control_path_rx_indication);
	if (!serial_ll_if_g) {
		printf("Serial interface creation failed\n\r");
		assert(serial_ll_if_g);
		return RET_FAIL;
	}
	if (RET_OK != serial_ll_if_g->fops->open(serial_ll_if_g)) {
		printf("Serial interface open failed\n\r");
		return RET_FAIL;
	}
	return RET_OK;
}

int control_path_platform_deinit(void)
{
	if (RET_OK != serial_ll_if_g->fops->close(serial_ll_if_g)) {
		printf("Serial interface close failed\n\r");
		return RET_FAIL;
	}
	return RET_OK;
}

static void control_path_rx_indication(void)
{
	/* heads up to control path for read */
	if(readSemaphore) {
		//osSemaphoreRelease(readSemaphore);
		g_h.funcs->_h_post_semaphore(readSemaphore);
	}
}


